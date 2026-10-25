These tests compile and execute the production `candidate_bbs_gicp_localizer.cpp`
with deterministic BBS/GICP return values. PCL loading/downsampling, Eigen, Polar
Context retrieval, candidate iteration, output JSON and gate logic remain real.
They verify orchestration and rejection behavior; they do not validate geometric
registration quality or replace replay of the failed field query.

Requirements: C++17 compiler, Eigen3, PCL headers and `pcl_io`, `pcl_filters`,
`pcl_common` libraries, pytest. Apply the reviewed BBS integration patch first.
The default source is the neighboring `agt_navigation_v3` checkout. To select a
checkout or native package explicitly:

```bash
BBS_NATIVE_SOURCE=/path/to/agt_navigation_v3 \
  python3 -m pytest -q tests/bbs_native/test_candidate_flow.py
```

Build products and synthetic inputs are written under pytest temporary paths.
The test process cannot contact ROS, publish poses, or command a robot.

`test_backend_process.py` extracts only `_run_backend` and `_stop_backend` from
the production ROS wrapper via Python's AST. It supplies fake ROS parameter access
while running real subprocesses, pipes and process-group termination. Its checks
cover successful exit, return code 4, timeout with preserved partial output,
SC cancellation, and refusing to spawn after stopping. It needs no PCL or ROS:

```bash
BBS_WRAPPER_SOURCE=/path/to/agt_navigation_v3 \
  python3 -m pytest -q tests/bbs_native/test_backend_process.py
```
