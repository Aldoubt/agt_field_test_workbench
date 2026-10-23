# 云端 ROS 2 Humble 软件验证

记录时间：2026-10-23 UTC。宿主机为 Debian 13；验证环境为官方 `ros:humble-ros-base-jammy` Docker 镜像（digest `sha256:566b87e6dfc48392189a9c32e594e84060fa9948de93b04e536f39475a1f6963`），Ubuntu 22.04 / ROS 2 Humble。工作区源码由本仓库的 15 个固定 SHA 检出；审计结果为 77 个不同的 `package.xml` 包名、重复 0、SHA 偏差 0、异常脏树 0。

容器中安装 `ros-humble-gtsam`、`ros-humble-sophus`，并通过 `rosdep` 安装可解析依赖；另外安装上游清单没有正确映射的 `libyaml-cpp-dev`、`libeigen3-dev`、`libasio-dev`。执行本仓库 `scripts/build_humble.sh`，**最终退出码 0**：Livox SDK2、3D-BBS CPU 和 AgileX UGV SDK 从固定源码编译；`colcon build` 的统一目标依赖图 **57 个包通过**。旧 `bunker_base`（与当前统一入口无关）和 `ugv_sdk` 的 ROS 1 catkin 声明被明确排除；统一入口使用的 `agt_bunker_base` 已编译通过。

Docker 以 root 访问宿主机普通用户持有的 Git 检出时，需要为固定源码仓库配置 Git `safe.directory`；工控机上以同一用户检出并编译则无需此设置。

软件测试：`colcon test --packages-select agt_pcd2grid_exporter agt_mapping_bringup --packages-ignore ugv_sdk bunker_base`，`colcon test-result --verbose` 汇总 **114 tests, 0 errors, 0 failures, 0 skipped**。`agt_field_test_bringup` 的 `colcon test` 命令通过，该包自身没有测试用例。`ros2 launch agt_field_test_bringup field_test.launch.py --show-args` 成功；Bunker 与 FW-mini 分别在 `enable_chassis:=false enable_mid360:=false enable_localization:=false enable_navigation:=false` 下启动了 `robot_state_publisher`，8 秒后以 SIGINT 正常退出。3D-BBS 的 `bbs_gicp_localizer`、`candidate_bbs_gicp_localizer` 文件已生成。

这次云端检查没有实车、CAN、MID360 数据或 rosbag。因此运动、外参、建图质量、全局重定位收敛、TF 唯一性、Nav2 路线都仍需工控机和现场验收；软件构建成功不表示这些项目通过。
