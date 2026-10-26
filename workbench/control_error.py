"""Control failures preserve their receipt identity across Unix RPC and HTTP."""
from copy import deepcopy


class ControlError(ValueError):
    def __init__(self, message, action=None, *, outcome_unknown=False, request_id=None):
        super().__init__(message)
        self.action = deepcopy(action) if isinstance(action, dict) else None
        self.outcome_unknown = bool(outcome_unknown)
        self.request_id = request_id if isinstance(request_id, str) else None

    def as_response(self):
        result = {'error': str(self)}
        if self.action is not None:
            result['action'] = deepcopy(self.action)
        if self.outcome_unknown:
            result['outcome_unknown'] = True
        if self.request_id is not None:
            result['request_id'] = self.request_id
        return result
