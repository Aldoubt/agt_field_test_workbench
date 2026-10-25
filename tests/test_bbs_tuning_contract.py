import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_bbs_tuning_patch_is_checksum_guarded_and_in_build_order():
    manifest = json.loads((ROOT / "patches/bbs_tuning_manifest.json").read_text())
    spec = manifest["agt_navigation_v3"]
    assert spec["commit"] == "b3a60487dff973c6326aca2ca2e9db1e78daeaaa"
    path = "navigation/localization/agt_global_relocalization/config/global_relocalization.yaml"
    hashes = spec["files"][path]
    assert re.fullmatch(r"[0-9a-f]{64}", hashes["before"])
    assert re.fullmatch(r"[0-9a-f]{64}", hashes["after"])

    build = (ROOT / "scripts/build_humble.sh").read_text()
    sc_kiss = build.index("apply_sc_kiss_integration.py")
    tuning = build.index("bbs_tuning_manifest.json")
    perception = build.index("navigation_perception_manifest.json")
    assert sc_kiss < tuning < perception

    patch = (ROOT / spec["patch"]).read_text()
    for expected in (
        "+      --candidate-top-k 4 --descriptor-prefilter 80",
        "+      --candidate-xy-radius 5.0 --candidate-z-radius 2.5",
        "+      --per-candidate-timeout 6.0 --bbs-score-threshold 0.03 --gicp-max-corr 2.5",
        "+    sdk_timeout_sec: 30.0",
    ):
        assert expected in patch
    # These post-refinement quality gates remain unchanged.
    assert "min_score: 0.50" not in patch
    assert "max_fitness: 1.00" not in patch
    assert "min_overlap: 0.20" not in patch


def test_range_documentation_distinguishes_lio_filter_from_det_range():
    doc = (ROOT / "docs/evidence/bbs_runtime_tuning_20261025/README.md").read_text()
    assert "lidar_max_range: 30.0" in doc
    assert "query_max_range_m: 30.0" in doc
    assert "det_range: 60" in doc
    assert "not** the input point-cloud maximum range" in doc
