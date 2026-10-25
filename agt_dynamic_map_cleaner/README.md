# agt_dynamic_map_cleaner (ROS 2 Humble)

Independent **offline** candidate map cleaner, separate from FAST-LIO2, its online
sliding map, LiDAR drivers, Qt render threads, and Nav2 controllers.

This package complements existing `workbench/map_cleanup.py` (world-voxel
persistence, near-field evidence, chassis filtering). That first stage is
retained. This second stage targets **start-of-run stationary people**, which
can otherwise pass temporal persistence tests.

## Preconditions

Use the outputs from a successful Bunker no-PGO temporal cleanup:

- `<mapping_run>/temporal_cleanup/clouds/static_map.pcd`
- `<mapping_run>/temporal_cleanup/keyframes/poses_timed.txt`
- `<mapping_run>/temporal_cleanup/keyframes/patches/*.pcd`
- `<mapping_run>/temporal_cleanup/manifest.json`

The CLI requires timestamped **body-frame patches** and map-frame poses from
the verified exact frame/pose association; it does not guess frame boundaries
inside a fused PCD. No motion and no ROS subscriptions are started.

```bash
source /opt/ros/humble/setup.bash
source ~/agt_field_test_ws/install/setup.bash
RUN=~/agt_field_test_ws/maps/<mapping_run_id>/temporal_cleanup
ros2 run agt_dynamic_map_cleaner clean_start \
  --static-pcd "$RUN/clouds/static_map.pcd" \
  --keyframes "$RUN/keyframes" \
  --cleanup-manifest "$RUN/manifest.json" \
  --config ~/agt_field_test_ws/src/agt_field_test_workbench/agt_dynamic_map_cleaner/config/start_clearance.yaml \
  --output "$RUN/start_clearance_candidate"
```

Outputs are **new immutable candidates**, never an active navigation map:

- `static_candidate.pcd`: original stage-1 static cloud minus well-supported
  start-area clearance candidates;
- `removed_candidates.pcd`: points selected for review, **not** labeled humans;
- `manifest.json`: input and output hashes, method, evidence counts, QA flags.

State remains `CANDIDATE` / `INSUFFICIENT_EVIDENCE`. No automatic map
promotion, route reuse, or BBS rebuild. If an operator later accepts this
candidate, generate PGM/YAML and BBS from the **same accepted static PCD** and
recheck all map/route/calibration SHA bindings.

## Evidence logic

At the start, identify elevated voxels within the configured radius which
were present in several distinct initial scans. During subsequent movement,
sample rays that extend past those voxels and hit returns farther away. Require
clearance from **multiple spatially distinct sensor viewpoints after the last
actual occupied observation**. Ground voxels are excluded. When no later
observations exist, leave every voxel unchanged. This deliberately does not
remove nearby vegetation/walls merely because a vehicle drove past them.

The conservative checks do NOT guarantee semantic-human classification. Blind
spots, pose drift, moving foliage, uncalibrated sensor origin, sparse rays, and
occluded places may yield `INSUFFICIENT_EVIDENCE`. The sensor ray origin is
currently the LIO body origin (close to MID360S center but not a separately
fitted optical center): verify on real bag before accepting candidates.
Use the compare view/removed cloud and reviewer judgement before publishing.

The algorithm uses bounded late-frame sampling and the field's existing
frame-index cloud archive. It runs offline; it cannot stall live LIO or Nav2.
