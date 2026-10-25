# 启动模式交付（2026-10-25）

导航阶段快照 `18f5c203a1405e11d24df79a1950c237243b4442` 已推送到 origin/feature/agri-sc-kiss-20261025，并创建、推送 annotated tag `field-long-distance-navigation-pass-20261025`。标签保留在组合入口开发之前，不移动。

后续新增 field_start distance/nav/photo，沿用原 field_trial 与 field_photo 组件；初始化只补齐缺失的根目录包装器。分终端方式保持可用，已有组件不会被新入口接管或杀死；组合模式发现旧链路时退出，继续使用旧命令即可。启动只进行停车初始化、地图和导航生命周期操作，不提交 Action 任务。photo 排队后仍由外部 `./field_photo start` 触发。

距离任务 teach/accuracy 读取或生成文件，run/home 显式下发任务，cancel 请求父任务取消。默认 5/7/10 米，也支持自定义 5/8/12 米；固定 home/目标，不自动循环、自动返程或填成功结果。距离执行器使用原精度到达判定，相机执行器使用拍照判定，两者不混用。

验证：90 项相关测试通过，见 tests.xml。包括三个组合模式及手动初始化的真实 DDS 服务测试（ROS_DOMAIN_ID=183，ROS_LOCALHOST_ONLY=1）、启动不下发任何 Action、旧分开组件冲突不改动、仅关闭自身进程组、组件失败不自动重启、发现服务期间移动时不请求重定位、距离原精度参数检查、外部显式触发、原拍照回中/取消/超时/返航、补丁初始化回放和既有现场配置回归。Python 编译、Bash 语法与三个真实配置 dry-run 通过。

真实 DDS 测试使用模拟的生命周期、定位、参数服务和 TF，组件进程也使用替身；没有启动/停止现场底盘、发送现场导航或云台目标、修改系统时钟服务或清空代价地图。组合入口仍需操作者在停车状态首次实机启动检查；该交付不新增实车精度或照片时间同步通过记录。

入口属于 Workbench 的 Python/Bash 脚本，复用已构建 ROS 包，无新增 C++/ROS 节点需要编译。导航依赖仍是 b3a60487dff973c6326aca2ca2e9db1e78daeaaa，FAST-LIO 仍是 7f664a66ae7b3b683dddf7f5322c1ea55a3c8141。检查两个修复 manifest 通过。依赖审计无 SHA 错误、无重复包；原有 bunker_ros2、fast_lio2_mapping 脏状态保留。

使用命令见 ../../launch_modes.md。地图、标定、bag、实拍图片留在原工作区，没有混入源码提交。
