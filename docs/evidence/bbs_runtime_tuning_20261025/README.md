# Field startup, obstacle clearing, and BBS tuning — 2026-10-25

## Which launch path actually uses the fix?

This repository does **not** contain a `nav.sh`. Its tracked entry points are:

- `./field_start nav` → `scripts/field_start.sh` → `workbench.field_start` → `field_test.launch.py`.
- `./field_trial start --global-backend bbs --hold-system-clock` → `scripts/field_trial.sh` → `workbench.field_trial` → `field_test.launch.py`.
- `scripts/run_field_test.sh` is the lower-level ROS launch wrapper.

The combined `field_start nav` defaults to BBS. For the split-terminal `field_trial` path, specify `--global-backend bbs` explicitly; the checked-in `config/field_trial.yaml` currently defaults to `sc_kiss`.

When `field_test.launch.py` starts navigation, it calls `agt_field_test_bringup.profiles.navigation()` and passes the generated `runtime_params_file` to the patched V3 navigation launch. For Bunker, that generator replaces the 3-D VoxelLayer with a 2-D ObstacleLayer: filtered obstacle points mark cells and the independent raw navigation cloud supplies clearing rays. The existing change also narrows the inflation radii to 0.65 m local / 0.80 m global. It does **not** expire obstacles while sensor data is missing; stale sensor data and Collision Monitor remain safety stops.

After startup, verify the active runtime rather than trusting the script name:

```bash
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /local_costmap/local_costmap obstacle_layer.lidar3d_mark.topic
ros2 param get /local_costmap/local_costmap obstacle_layer.lidar3d_clear.topic
```

Expected plugin list contains `obstacle_layer` and `inflation_layer`, with separate mark and clear topics. If those node names do not exist, inspect `ros2 node list`: a separately started older navigation launch may still own the running system.

The 2026-10-25 costmap A/B evidence is in [costmap clearing](../costmap_clearing_20261025/README.md): the native-plugin isolated test and 106 regression tests passed. The fix still needs a supervised real-robot check.

## BBS candidate-search tuning

The pinned V3 config was too narrow for an exploratory field trial: top-k 2, descriptor prefilter 40, XY/Z candidate radii 4.0/2.0 m, BBS score threshold 0.05, and per-candidate timeout 8 s. The workbench now applies a checksum-guarded patch with:

- Candidate top-k: **2 → 4**; descriptor prefilter: **40 → 80**.
- Candidate search radius: **4.0/2.0 → 5.0/2.5 m (XY/Z)**.
- BBS score threshold: **0.05 → 0.03**; per-candidate timeout: **8 → 6 s**.
- GICP maximum correspondence distance: **2.0 → 2.5 m**.
- Default total SDK budget: **18 → 30 s**.

The lower BBS score threshold and broader candidate set allow a wider coarse search; the 6 s per-candidate budget bounds individual attempts. The GICP change is aimed at the observed `small_gicp did not converge after candidate-guided BBS` failure. This is a **trial tuning**, not proof of improved localization. Do not loosen the ROS-side quality gates (`min_score=0.50`, `max_fitness=1.00`, `min_overlap=0.20`): those are already permissive, and the observed failure occurred during GICP convergence rather than those gates. Compare the same bag/query, retain false-acceptance checks, and do not accept a pose solely because the matcher returns success.

## The 60 m / 40 m question

There is no active 60 m → 40 m point-cloud clipping change in the current workbench configuration. The pinned FAST-LIO2 source config has `lidar_max_range: 30.0`; the global relocalizer has `query_max_range_m: 30.0`. These are the actual point-range filters currently used by the LIO and relocalization query respectively.

The same FAST-LIO2 YAML has `det_range: 60`, but that is the local-map detection/cube-shift parameter, **not** the input point-cloud maximum range. Do not change `det_range` to 40 expecting it to clip points. The LIO and relocalization query are currently aligned at 30 m; keep that alignment for this test because the source map was built under the same range contract. If a deliberate 40 m query is desired later, change and validate both LIO input filtering and query filtering against the selected map, rather than only changing one number.

## Night-of-test commands

For the current workbench checkout:

```bash
# BBS + supervised field startup; keep the robot stationary during initialization
./field_start nav
# Or, split-terminal startup with explicit backend:
./field_trial start --global-backend bbs --hold-system-clock
```

After applying the branch and rebuilding the affected packages, check the active BBS parameters:

```bash
ros2 param get /agt_global_relocalization global_backend
ros2 param get /agt_global_relocalization sdk_timeout_sec
ros2 param get /agt_global_relocalization query_max_range_m
ros2 param get /agt_global_relocalization candidate_sdk_command
```

Do not drive until the normal localization, TF, costmap and guarded-command readiness checks pass. This change does not authorize motion or waive the human-stop check.
