"""Small localhost client for the existing Field Test Workbench API.

The Qt application deliberately reuses map versions, sessions, relocalization and
NavigateToPose instead of creating a second ROS backend.
"""
import json
from copy import deepcopy
import math
import queue
import threading
import time
import uuid
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from PyQt5.QtCore import QThread, pyqtSignal


def _action_metadata(value):
    """Keep correlation fields only; never surface a command payload in telemetry."""
    if not isinstance(value, dict):
        return None
    allowed = {'bridge_id', 'action_id', 'request_id', 'kind', 'state', 'terminal',
               'seq', 'updated_at', 'reason', 'map_id', 'route_id', 'robot'}
    result = {}
    for key in allowed.intersection(value):
        item = value[key]
        if item is None or isinstance(item, (bool, int)):
            result[key] = item
        elif isinstance(item, float) and math.isfinite(item):
            result[key] = item
        elif isinstance(item, str):
            result[key] = item[:2048 if key == 'reason' else 512]
    return result


class WorkbenchError(RuntimeError):
    """Backward-compatible text error with optional transport/action metadata."""
    def __init__(self, message, *, kind, method, path, status_code=None, action=None,
                 outcome_unknown=False, request_id=None):
        super().__init__(message)
        self.kind = kind
        self.method = method
        self.path = path
        self.status_code = status_code
        self.action = _action_metadata(action)
        self.outcome_unknown = outcome_unknown is True
        self.request_id = request_id if isinstance(request_id, str) and 0 < len(request_id) <= 128 else None


def _timeout(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Workbench timeout must be finite and positive')
    return value


class WorkbenchClient:
    def __init__(self, base_url='http://127.0.0.1:8765', timeout=8.0):
        self.base_url = base_url.rstrip('/')
        self.timeout = _timeout(timeout)

    def request(self, method, path, payload=None, timeout=None):
        if not path.startswith('/api/'):
            raise ValueError('Workbench API path must start with /api/')
        method = method.upper()
        request_timeout = self.timeout if timeout is None else _timeout(timeout)
        command_request_id = payload.get('request_id') if isinstance(payload, dict) else None
        data = None if payload is None else json.dumps(payload).encode('utf8')
        request = Request(self.base_url + path, data=data, method=method)
        if data is not None:
            request.add_header('Content-Type', 'application/json')
        try:
            with urlopen(request, timeout=request_timeout) as response:
                return json.loads(response.read().decode('utf8'))
        except HTTPError as exc:
            action = None
            outcome_unknown = False
            error_request_id = command_request_id
            try:
                body = json.loads(exc.read().decode('utf8'))
                detail = body.get('error', str(exc)) if isinstance(body, dict) else str(exc)
                action = body.get('action') if isinstance(body, dict) else None
                if isinstance(body, dict):
                    outcome_unknown = body.get('outcome_unknown') is True
                    error_request_id = body.get('request_id', command_request_id)
            except Exception:
                detail = str(exc)
            raise WorkbenchError(str(detail), kind='http', method=method, path=path,
                                 status_code=exc.code, action=action, outcome_unknown=outcome_unknown,
                                 request_id=error_request_id) from exc
        except (URLError, TimeoutError, OSError) as exc:
            timed_out = isinstance(exc, TimeoutError) or isinstance(getattr(exc, 'reason', None), TimeoutError)
            raise WorkbenchError(('Workbench 请求超时：' if timed_out else 'Workbench 未连接：') + str(exc),
                                 kind='timeout' if timed_out else 'connection', method=method, path=path,
                                 outcome_unknown=method not in ('GET', 'HEAD'),
                                 request_id=command_request_id) from exc
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise WorkbenchError('Workbench 响应不是有效 JSON：' + str(exc),
                                 kind='response', method=method, path=path,
                                 outcome_unknown=method not in ('GET', 'HEAD'),
                                 request_id=command_request_id) from exc

    def get(self, path):
        return self.request('GET', path)

    def post(self, path, payload=None):
        return self.request('POST', path, payload or {})


class ApiWorker(QThread):
    """One FIFO lane; never retry or reorder accepted commands.

    ``completed`` retains the original payload/string contract. ``request_event``
    adds transport timing and safe error correlation, without command payloads.
    A request timeout bounds socket operations, not total server execution: a
    timed-out POST may already have run and must never be retried automatically.
    """
    completed = pyqtSignal(str, bool, object)
    request_event = pyqtSignal(object)

    def __init__(self, client, *, max_queue=128, coalesce_get=False):
        super().__init__()
        if isinstance(max_queue, bool) or not isinstance(max_queue, int) or max_queue < 1:
            raise ValueError('max_queue must be a positive integer')
        self.client = client
        self.requests = queue.Queue(maxsize=max_queue)
        self.alive = True
        self.active_request = None
        self.coalesce_get = bool(coalesce_get)
        self._active = None
        self._pending = {}
        self._get_pending = set()
        self._pending_lock = threading.RLock()
        self._last_completed = None
        self._coalesced = 0
        self._event_seq = 0

    def status_snapshot(self):
        """Thread-safe lane state; times are monotonic durations, not ROS time."""
        with self._pending_lock:
            active = self._active
            return dict(closed=not self.alive, active_request=self.active_request,
                        active_path=active['path'] if active else None,
                        active_elapsed_sec=max(0., time.monotonic()-active['started']) if active else 0.,
                        queued=self.requests.qsize(), pending=sum(self._pending.values()),
                        queue_capacity=self.requests.maxsize, coalesced=self._coalesced,
                        last_completed=deepcopy(self._last_completed))

    def _event(self, request, state, **extra):
        # Allocate sequence numbers under the state lock, but emit outside it:
        # direct Qt slots may re-enter the event loop or request shutdown.
        self._event_seq += 1
        event = dict(request_id=request['id'], name=request['name'], method=request['method'],
                     path=request['path'], state=state, seq=self._event_seq,
                     queue_depth=self.requests.qsize(), **extra)
        return event

    def _publish(self, event):
        self.request_event.emit(deepcopy(event))

    def _remove_pending(self, request):
        name = request['name']
        remaining = self._pending.get(name, 0) - 1
        if remaining > 0:
            self._pending[name] = remaining
        else:
            self._pending.pop(name, None)
        if request['method'] == 'GET':
            self._get_pending.discard((name, request['path']))

    def has_pending(self, names):
        with self._pending_lock:
            return any(self._pending.get(name, 0) > 0 for name in names)

    def call(self, name, method, path, payload=None, timeout=None):
        method = method.upper()
        request = dict(id=uuid.uuid4().hex, name=name, method=method,
                       path=path if method != 'LOCAL' else '<local>', callback=path if method == 'LOCAL' else None,
                       payload=None, timeout=timeout, queued=time.monotonic())
        try:
            request['payload'] = deepcopy(payload)
        except Exception:
            return self._reject(request, 'invalid_request', '请求数据无法复制，未发送')
        rejected, event, accepted = None, None, False
        with self._pending_lock:
            if not self.alive:
                rejected = ('closed', '操作队列已关闭，请重新连接')
            elif self.coalesce_get and method == 'GET' and (name, request['path']) in self._get_pending:
                self._coalesced += 1
                event = self._event(request, 'coalesced')
            else:
                try:
                    self.requests.put_nowait(request)
                except queue.Full:
                    rejected = ('queue_full', '操作队列已满，请稍后重试')
                else:
                    accepted = True
                    self._pending[name] = self._pending.get(name, 0) + 1
                    if method == 'GET':
                        self._get_pending.add((name, request['path']))
                    event = self._event(request, 'queued')
        if rejected:
            return self._reject(request, *rejected)
        self._publish(event)
        return accepted

    def _reject(self, request, kind, message):
        with self._pending_lock:
            event = self._event(request, 'rejected', success=False, error_kind=kind, error=message)
        self._publish(event)
        self.completed.emit(request['name'], False, message)
        return False

    def local(self,name,callback):
        return self.call(name,'LOCAL',callback)

    def run(self):
        while True:
            try:
                request = self.requests.get(timeout=.1)
            except queue.Empty:
                if not self.alive:
                    return
                continue
            with self._pending_lock:
                if not self.alive:
                    event = self._cancel(request)
                    cancelled = True
                else:
                    cancelled = False
                    name, method, path = request['name'], request['method'], request['path']
                    request['started'] = time.monotonic()
                    self._active = request
                    self.active_request = name
                    queue_wait = request['started']-request['queued']
                    event = self._event(request, 'running', queue_wait_sec=queue_wait)
            self._publish(event)
            if cancelled:
                self.completed.emit(request['name'], False, event['error'])
                continue
            success, error = False, None
            try:
                result = request['callback']() if method=='LOCAL' else self.client.request(
                    method, path, request['payload'], timeout=request['timeout'])
                if name in ('map_diff','map_diff_preview') and isinstance(result,dict):
                    import base64
                    from PyQt5.QtGui import QImage
                    result['_overlay_image']=QImage.fromData(base64.b64decode(result.get('overlay_png_b64','')))
                success = True
            except Exception as exc:
                result, error = str(exc), exc
            finally:
                with self._pending_lock:
                    now = time.monotonic()
                    extra = dict(success=success, queue_wait_sec=queue_wait,
                                 elapsed_sec=now-request['started'], total_elapsed_sec=now-request['queued'])
                    if error is not None:
                        extra.update(error_kind=getattr(error, 'kind', 'internal'), error=str(error)[:2048],
                                     http_status=getattr(error, 'status_code', None),
                                     error_action=deepcopy(getattr(error, 'action', None)),
                                     outcome_unknown=getattr(error, 'outcome_unknown', False),
                                     error_request_id=getattr(error, 'request_id', None))
                    self._remove_pending(request)
                    self.active_request = None
                    self._active = None
                    event = self._event(request, 'completed', **extra)
                    self._last_completed = event
                self.requests.task_done()
            self._publish(event)
            self.completed.emit(name, success, result)

    def _cancel(self, request):
        """Cancel only requests not yet handed to the HTTP client."""
        self._remove_pending(request)
        event = self._event(request, 'cancelled', success=False, error_kind='closed',
                            error='界面关闭：排队操作未发送')
        self.requests.task_done()
        return event

    def shutdown(self, wait_ms=125000):
        """Reject new work and cancel queued work; return whether QThread ended.

        An active HTTP request is allowed to finish and keeps its true result.
        False means the owner MUST retain this object and defer window disposal;
        it does not mean the server cancelled the operation. No terminate/retry.
        """
        cancelled = []
        with self._pending_lock:
            self.alive = False
            while True:
                try:
                    cancelled.append(self._cancel(self.requests.get_nowait()))
                except queue.Empty:
                    break
        for event in cancelled:
            self._publish(event)
            self.completed.emit(event['name'], False, event['error'])
        return self.wait(max(0, int(wait_ms)))


class TelemetryWorker(ApiWorker):
    """Separate, bounded read-only lane for small live/jobs/components GETs.

    Large map/cloud GETs belong to the asset lane. Duplicate pending name/path
    polls share the outstanding result. This is coalescing, never a retry.
    """
    def __init__(self, client, *, max_queue=16, timeout=2.0):
        super().__init__(client, max_queue=max_queue, coalesce_get=True)
        self.timeout = _timeout(timeout)

    def call(self, name, method, path, payload=None, timeout=None):
        if method.upper() != 'GET':
            request = dict(id=uuid.uuid4().hex, name=name, method=method.upper(), path='<local>' if method.upper() == 'LOCAL' else path)
            return self._reject(request, 'read_only', '遥测队列只允许 GET，请使用原有控制队列')
        return super().call(name, 'GET', path, payload,
                            self.timeout if timeout is None else min(self.timeout, _timeout(timeout)))
