# AGT Field Test Workbench

单一 **Ubuntu 22.04 / ROS 2 Humble** 工作区，支持 Bunker / FW-mini。当前提供网页选择底盘和外参、无 PGO 建图与 raw bag、全量 PCD 清洗、2D 编辑、地图绑定路线、受监督 jog 和有门禁的 Nav2 距离目标。

**实车验收未执行。** 软件测试、合成 DDS/点云测试、真实 bag 和物理测量分别记录；没有数据的项目保持 `NOT_RUN`。Qt 工作台默认使用 Scan Context + KISS-Matcher + small_gicp，3D-BBS 保留为显式备用；这不代表真实 bag 或实车定位已验收。GPS 默认仅观察。默认不启动底盘、不进入运动档、不启用自主导航。

## 工控机首次部署

```bash
mkdir -p ~/agt_field_test_ws/src
git clone https://github.com/Aldoubt/agt_field_test_workbench.git ~/agt_field_test_ws/src/agt_field_test_workbench
cd ~/agt_field_test_ws/src/agt_field_test_workbench
sudo apt-get install -y python3-rosdep python3-colcon-common-extensions \
  python3-scipy python3-setuptools libpcl-dev libtbb-dev ros-humble-gtsam ros-humble-sophus \
  libyaml-cpp-dev libeigen3-dev libasio-dev
# 首次使用 rosdep 才需要 sudo rosdep init
rosdep update --rosdistro humble
bash scripts/init_workspace.sh
bash scripts/build_humble.sh
bash scripts/run_workbench.sh
```

打开 `http://127.0.0.1:8765`。远程使用 `ssh -L 8765:127.0.0.1:8765 <工控机>`。日常操作使用一个网页，无需拼多个终端。升级现有工作区先检查 `git status`，保留现场修改和 `calibration/`；初始化仅补充缺失模板，遇到不匹配的锁或补丁停止并保留文件。

- [2026-10-24 建图修复、去残影前后图与代码分支](docs/evidence/mapping_cleanup_20261024/README.md)
- [逐页操作手册与实机分阶段步骤](docs/workbench_operator_manual.md)
- [本轮交付、软件证据、尚未执行的验收](docs/field_loop_delivery.md)
- [历史交付记录](docs/field_delivery.md)
- [FW-mini 原始外参证据与验证升级](docs/fwmini_mid360_integration.md)
- [CtrlCmd 6/8 状态机与实机架轮步骤](docs/fwmini_ctrl_cmd_delivery.md)

## 实现约定

- 17 个主依赖固定 SHA；可选 KISS Python 构建额外锁定 pybind11、ROBIN 归档、PMC、atomic_queue、xenium。[依赖锁](dependencies/field_test.repos) / [可选构建锁](dependencies/kiss_python_build.json)。原仓库 HEAD 不变，V3/FW-mini 的改动由精确补丁和前后 SHA 校验交付。
- 当前机器人 profile 同时生成 controller、smoother、guard 和驱动参数。Bunker 默认上限 0.15 m/s；FW-mini 0.05 m/s。手动按住续租，松手或失焦归零；反馈/控制权失效清除意图，恢复后须新命令。
- 默认 FAST-LIO2 + 独立 `lio_artifact` 导出器，**不启动 PGO**。PGO 仅显式离线实验。live raw 输入录制在时间修正前；导出后自动进入 DRAFT 地图版本。
- PCL 全分辨率处理保留原始 PCD，所有清洗、转换、2D 编辑和路线保存新版本。仅编辑 PGM 像素且原点/分辨率不变时，保留同源三维 PCD 与 SC/BBS 资产；更换 PCD 或地图坐标几何才需要重建定位资产。预览采样不冒充清洗；DRAFT 可编辑导出。
- RSP 唯一发布物理安装边；轮式 odom 不发布 TF；选中的 LIO adapter 发布局部运动；Localization Manager 唯一发布 `map→odom`。GPS gate 不发布 TF。
- Nav2 必须绑定 ACCEPTED 二维导航地图、VERIFIED 外参、物理安全报告，并具备新鲜定位、底盘反馈和 ACTIVE 生命周期。像素编辑后需重新保存/校验路线；原物理报告仅在同源 PCD、地图坐标几何与标定不变时可复用。FW-mini 当前 RPP 自主链只接受 6 档，8 档用于受监督横移；横移自主控制器尚未验收。
- 网页只调用固定的 ROS 服务/Action 或固定处理任务，没有任意 shell 接口。路线拍照保留 task executor 接口；未配置相机执行器时拒绝执行拍照路线。

云端已部署真实 Humble 容器。软件构建/测试通过不表示物理制动、RTK、急停、遥控接管或导航精度通过。

## 20261024 automation delivery

三视图实时工作台、C1 采集、组件独立生命周期与地图差异人工审核的实现、测试结果和上车验收顺序见
[交付记录](docs/field_automation_20261024_delivery.md)。真实硬件验收状态为 `HARDWARE_VERIFIED: NOT_RUN`。

## 命令行与 RViz 现场入口

保留分终端启动：见 [CLI 导航](docs/cli_rviz_field_trial.md)、[多点云台拍照](docs/rviz_photo_tasks.md)、[距离精度测试](docs/accuracy_and_photo_tasks.md)。`./field_photo start` 仅在操作者明确执行时触发队列任务。

[长距离导航阶段快照与测试边界](docs/evidence/long_distance_navigation_20261025/README.md)。依赖保持锁定，现场修复由 patches/ 中带 SHA 校验的补丁重放。
