import json
import math
import pytest
pytest.importorskip('PyQt5')
from agt_robot_studio.ui import pixel_to_world
from agt_robot_studio.workbench_client import WorkbenchClient


def test_pixel_to_world_preserves_origin_yaw_and_pgm_flip():
    x, y = pixel_to_world(0, 1, 2, 2, .5, (1., 2., math.pi/2))
    assert math.isclose(x, 1. - .25)
    assert math.isclose(y, 2. + .25)


def test_workbench_client_uses_fixed_local_api(monkeypatch):
    seen = {}
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self): return json.dumps({'ready': True}).encode()
    def open(request, timeout):
        seen.update(method=request.get_method(), path=request.full_url, timeout=timeout)
        return Response()
    monkeypatch.setattr('agt_robot_studio.workbench_client.urlopen', open)
    client = WorkbenchClient('http://127.0.0.1:8765/')
    assert client.get('/api/live') == {'ready': True}
    assert seen == {'method': 'GET', 'path': 'http://127.0.0.1:8765/api/live', 'timeout': 8.0}
