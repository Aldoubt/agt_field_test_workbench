"""Small localhost client for the existing Field Test Workbench API.

The Qt application deliberately reuses map versions, sessions, relocalization and
NavigateToPose instead of creating a second ROS backend.
"""
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from PyQt5.QtCore import QThread, pyqtSignal
import queue


class WorkbenchClient:
    def __init__(self, base_url='http://127.0.0.1:8765', timeout=8.0):
        self.base_url = base_url.rstrip('/')
        self.timeout = float(timeout)

    def request(self, method, path, payload=None, timeout=None):
        if not path.startswith('/api/'):
            raise ValueError('Workbench API path must start with /api/')
        data = None if payload is None else json.dumps(payload).encode('utf8')
        request = Request(self.base_url + path, data=data, method=method)
        if data is not None:
            request.add_header('Content-Type', 'application/json')
        try:
            with urlopen(request, timeout=self.timeout if timeout is None else float(timeout)) as response:
                return json.loads(response.read().decode('utf8'))
        except HTTPError as exc:
            try:
                detail = json.loads(exc.read().decode('utf8')).get('error', str(exc))
            except Exception:
                detail = str(exc)
            raise RuntimeError(detail) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise RuntimeError('Workbench 未连接：' + str(exc)) from exc

    def get(self, path):
        return self.request('GET', path)

    def post(self, path, payload=None):
        return self.request('POST', path, payload or {})


class ApiWorker(QThread):
    """Serialize localhost API work without blocking Qt's GUI thread."""
    completed = pyqtSignal(str, bool, object)

    def __init__(self, client):
        super().__init__()
        self.client = client
        self.requests = queue.Queue(maxsize=128)
        self.alive = True

    def call(self, name, method, path, payload=None, timeout=None):
        try:
            self.requests.put_nowait((name, method, path, payload, timeout))
        except queue.Full:
            self.completed.emit(name, False, '操作队列已满，请稍后重试')

    def run(self):
        while self.alive:
            try:
                name, method, path, payload, timeout = self.requests.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                result = self.client.request(method, path, payload, timeout=timeout)
                self.completed.emit(name, True, result)
            except Exception as exc:
                self.completed.emit(name, False, str(exc))

    def shutdown(self):
        self.alive = False
        # A recorder start/stop or Stop All may legitimately wait for ROS process
        # groups. Do not destroy the QThread while its current urllib call is alive.
        self.wait(125000)
