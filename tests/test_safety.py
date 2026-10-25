import json

import pytest

from workbench.safety import CHECKS, report


def test_physical_report_can_be_reused_for_grid_pixel_edit_only(tmp_path):
    record={
        'scope':'PHYSICAL_NAVIGATION','robot':'bunker','profile_sha256':'profile-sha',
        'map_sha256':'source-grid-sha','calibration_sha256':'calibration-sha',
        'operator':'field operator','observed_at':'2026-10-25T08:00:00Z',
        'observations':['same PCD, same map-frame geometry, pixel-only planner edit'],
        'checks':{key:'PASS' for key in CHECKS},
    }
    path=tmp_path/'safety-report.json'
    path.write_text(json.dumps(record))
    result=report(path,'bunker','profile-sha','edited-grid-sha','calibration-sha',
                  compatible_map_sha256s=['source-grid-sha'])
    assert result['data']['map_sha256']=='source-grid-sha'
    with pytest.raises(ValueError,match='PHYSICAL_NAVIGATION'):
        report(path,'bunker','profile-sha','edited-grid-sha','calibration-sha')
