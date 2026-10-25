# Studio field handoff: local-ground grid and Bunker tuning

## Why the converter was wrong

The old Workbench `agt_field_test_bringup/pcd_ops` grid option classified
obstacles and free ground by **absolute FAST-LIO world Z**, although the mapping
world frame is `lidar` rather than a ground-level frame. This was a new,
simplified implementation; it never called the existing mapping repository's
`agt_pcd2grid_exporter`.

As of this integration branch, `workbench.artifacts.Clouds.grid()` invokes the
**pinned** `agt-lio-pgo-mapping/exporters/agt_pcd2grid_exporter` binary with
`--pcd` and `projection_mode: local_ground`. It uses local elevation
percentiles and **relative** obstacle heights, not the global Z height slice.
The original localization PCD is not changed. Output retains:

- the exported `map.pgm`, `map.yaml`, `projection.yaml`, and `metadata.yaml`;
- a Workbench immutable `maps/versions/<id>/` DRAFT version;
- original cloud ID, PCD SHA, calibration/bag provenance, converter ID, effective
  projection config SHA, and generation statistics.

Unobserved stays **unknown**. A stationary scan with few ground hits may still
contain little free space. There is no automatic acceptance, no obstacle
safety claim, and no silent change of the localization reference map.

**Limit:** the older `traversability` converter additionally needs
`--package` with `patches/*.pcd` and `poses_timed.txt`; a fused no-PGO
PCD alone does not contain this input evidence. Use `local_ground` for the
current FAST-LIO2 PCD. Do not fabricate free space or claim ray-carving.

Old Studio user configs containing `z_min`, `z_max`, `ground_min` are read,
but those legacy **absolute-Z** fields are intentionally ignored by the new
converter instead of silently being treated as relative heights. After opening
the updated Studio, save the new local-ground parameters once.

## Update the field workstation

The remote feature branch may be ahead of the local copy. In the workbench
repository, stash your local layout/timeout edits **before** pulling if they
are not yet pushed:

```bash
cd ~/agt_field_test_ws/src/agt_field_test_workbench
git status --short
git stash push -u -m "local-ui-before-grid-integration-$(date +%Y%m%d_%H%M%S)"
git fetch origin '+refs/heads/feature/agt-robot-studio-v1:refs/remotes/origin/feature/agt-robot-studio-v1'
git switch feature/agt-robot-studio-v1
git merge --ff-only origin/feature/agt-robot-studio-v1
```

No V3 change is needed for this **offline PCD-to-PGM** replacement.
The full V3 dependency-lock/control-patch reconciliation remains separate and
is **not** validated by the map change.

Verify the existing exporter (do not invoke the old fixed-height `pcd_ops`
projection for this test):

```bash
cd ~/agt_field_test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix agt_pcd2grid_exporter
ros2 run agt_pcd2grid_exporter pcd2grid_exporter --help
colcon build --base-paths src --packages-select agt_robot_studio \
  --packages-ignore ugv_sdk bunker_base --symlink-install
```

If the exporter is missing, build **just** the mapped exporter using
`colcon build --base-paths src --packages-select agt_pcd2grid_exporter
--packages-ignore ugv_sdk bunker_base --symlink-install`, then re-source
`install/setup.bash`. Do not rerun the incompatible whole-workspace lock
script just for projection.

Run `python3 -m pytest -q tests/test_local_ground_workbench.py` in the
Workbench repository with the Humble overlay sourced. These are mocked
interface tests, **not** measured real-PCD acceptance.

## Bunker controller parameter workflow (no automatic motion)

The Studio **参数 → Bunker 运动限制与响应参数** card uses the existing
Workbench `GET /api/profiles` and `POST /api/profiles/bunker` interface.

1. End the active sensor/manual/navigation session normally; saving with an
   active session is rejected by the backend.
2. Read the current Bunker profile, change **one parameter at a time**,
   and save explicitly. The backend creates an immutable
   `config/robots/bunker_<timestamp>.yaml` and an
   `active_bunker.json` pointer with SHA-256. It does **not** issue velocity
   commands or silently modify any running process.
3. The next manually supervised session takes the selected profile. Record
   the version SHA, test only with a verified physical E-stop/remote takeover,
   and keep existing motion safety gates. Start with low limits.

Current editable keys: `forward_mps`, `reverse_mps`, `angular_radps`,
`linear_accel_mps2`, `linear_decel_mps2`,
`angular_accel_radps2`, `angular_decel_radps2`,
`command_timeout_sec`.

**Not exposed/changed here:** Bunker low-level motor PID, Nav2 controller
lookahead / regulated-pure-pursuit gains, feedback correction, steering
kinematics, CAN authority, wheel feedback validity, safety policy. Profile
limit tuning alone cannot prove or fix left-right oscillation. Track
commanded velocity, actual chassis motion and time alignment before tuning
Nav2 tracking gains.

## Known UI limitations

The PCD-to-PGM HTTP endpoint still runs synchronously, and the Studio requests
now allow up to 600 seconds instead of 8 seconds. This is a stopgap:
do not repeatedly press Generate while a request is in flight. The next
architecture iteration should make projection a Workbench job with job ID,
progress, deduplication and polling-based result recovery.

Do **not** mark the static, unverified grid ACCEPTED, nor use it for motion.
