"""Thread-safe command acknowledgements, independent of ROS and robot state.

DISPATCHED means that a command was sent. It does not establish physical motion,
arrival, or a stopped chassis. Retained request identities prevent duplicate
movement dispatch for the lifetime of this bridge.
"""
from collections import OrderedDict
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import re
import threading
import uuid


MOVEMENT_KINDS = frozenset({'goal', 'route', 'distance'})
TERMINAL_STATES = frozenset({'SUCCEEDED', 'FAILED', 'REJECTED', 'CANCELED', 'DISPATCHED'})
ACTIVE_ORDER = {'REQUESTED': 0, 'ARMING': 1, 'ACCEPTED': 2, 'RUNNING': 3,
                'CANCEL_REQUESTED': 4, 'STOP_REQUESTED': 4}
STATES = TERMINAL_STATES | ACTIVE_ORDER.keys()
REQUEST_LIMIT = 4096
RETAIN_RECORDS = 128
RECENT_RECORDS = 32
_IDENTIFIER = re.compile(r'[A-Za-z0-9_.:-]{1,128}\Z')
_RESERVED = frozenset({'bridge_id', 'action_id', 'request_id', 'kind', 'state',
                       'terminal', 'seq', 'created_at', 'updated_at', 'reason', 'payload'})
_SUMMARY_FIELDS = _RESERVED - {'payload'} | {'context', 'map_id', 'robot', 'route_id'}


def _now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def _json_copy(value):
    # Reject non-finite numbers and values that cannot cross the JSON bridge.
    try:
        serialized = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError('action data must contain finite JSON values') from exc
    return deepcopy(value), serialized


class ActionLedger:
    """Track acknowledgements and retain request-id tombstones until restart.

    Full records retain the latest 128 entries, plus every nonterminal action.
    Evicted terminal records retain an immutable summary and their fingerprint.
    At 4096 unique requests, new movement requests fail closed; safety commands
    such as stop can still be recorded. Those request tombstones also remain
    valid until bridge restart, rather than silently permitting redispatch.
    """

    def __init__(self):
        self.bridge_id = uuid.uuid4().hex
        self._lock = threading.RLock()
        self._sequence = 0
        self._records = OrderedDict()
        self._requests = {}
        self._terminal_summaries = {}

    @property
    def busy(self):
        return self.busy_except(None)

    def busy_except(self, action_id):
        with self._lock:
            return any(record['action_id'] != action_id and
                       record['kind'] in MOVEMENT_KINDS and not record['terminal']
                       for record in self._records.values())

    def get(self, action_id):
        with self._lock:
            record = self._records.get(action_id, self._terminal_summaries.get(action_id))
            return deepcopy(record) if record is not None else None

    def for_request(self, request_id):
        """Read an existing receipt without allocating or dispatching anything."""
        with self._lock:
            previous = self._requests.get(request_id) if isinstance(request_id, str) else None
            return self.get(previous[0]) if previous else None

    def begin(self, kind, payload=None):
        if not isinstance(kind, str) or not _IDENTIFIER.fullmatch(kind):
            raise ValueError('action kind must be an identifier of 1 to 128 characters')
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ValueError('action payload must be an object')
        copied, _ = _json_copy(payload)
        request_id = copied.get('request_id', uuid.uuid4().hex)
        if not isinstance(request_id, str) or not _IDENTIFIER.fullmatch(request_id):
            raise ValueError('request_id must match [A-Za-z0-9_.:-] and contain 1 to 128 characters')
        copied['request_id'] = request_id
        _, canonical = _json_copy(copied)
        fingerprint = hashlib.sha256((kind + '\n' + canonical).encode('utf-8')).hexdigest()
        context = copied.get('context', {})
        if not isinstance(context, dict):
            raise ValueError('action context must be an object')
        with self._lock:
            previous = self._requests.get(request_id)
            if previous is not None:
                action_id, old_fingerprint = previous
                if old_fingerprint != fingerprint:
                    raise ValueError('request_id is already associated with a different action payload')
                return self.get(action_id), False
            if kind in MOVEMENT_KINDS and len(self._requests) >= REQUEST_LIMIT:
                raise ValueError('action request limit reached; restart the bridge before new movement')
            self._sequence += 1
            action_id = uuid.uuid4().hex
            now = _now()
            record = dict(bridge_id=self.bridge_id, action_id=action_id,
                          request_id=request_id, kind=kind, state='REQUESTED',
                          terminal=False, seq=self._sequence,
                          created_at=now, updated_at=now, reason='',
                          payload=copied, context=deepcopy(context))
            for name in ('map_id', 'robot', 'route_id'):
                if name in copied or name in context:
                    record[name] = deepcopy(copied.get(name, context.get(name)))
            self._records[action_id] = record
            self._requests[request_id] = (action_id, fingerprint)
            self._trim()
            return deepcopy(record), True

    def update(self, action_id, state, reason='', **fields):
        if state not in STATES:
            raise ValueError('unknown action state: ' + str(state))
        if not isinstance(reason, str):
            raise ValueError('action reason must be a string')
        if _RESERVED.intersection(fields):
            raise ValueError('action identity and lifecycle fields cannot be overwritten')
        copied, _ = _json_copy(fields)
        with self._lock:
            record = self._records.get(action_id, self._terminal_summaries.get(action_id))
            if record is None:
                raise KeyError(action_id)
            # Results and cancellation are monotonic: late goal-response and
            # feedback callbacks cannot resurrect a finished/canceling action.
            if record['terminal']:
                return deepcopy(record)
            if state in ACTIVE_ORDER and ACTIVE_ORDER[state] < ACTIVE_ORDER[record['state']]:
                return deepcopy(record)
            self._sequence += 1
            record.update(copied)
            record.update(state=state, terminal=state in TERMINAL_STATES,
                          reason=reason, seq=self._sequence, updated_at=_now())
            self._records.move_to_end(action_id)
            self._trim()
            return deepcopy(record)

    def _trim(self):
        if len(self._records) <= RETAIN_RECORDS:
            return
        for action_id, record in list(self._records.items()):
            if len(self._records) <= RETAIN_RECORDS:
                break
            if record['terminal']:
                self._terminal_summaries[action_id] = {
                    key: deepcopy(value) for key, value in record.items() if key in _SUMMARY_FIELDS}
                del self._records[action_id]

    def snapshot(self):
        with self._lock:
            records = list(reversed(self._records.values()))
            current = next((record for record in records
                            if record['kind'] in MOVEMENT_KINDS and not record['terminal']), None)
            return deepcopy(dict(bridge_id=self.bridge_id, sequence=self._sequence,
                                 busy=current is not None, current=current,
                                 last=records[0] if records else None,
                                 recent=records[:RECENT_RECORDS]))
