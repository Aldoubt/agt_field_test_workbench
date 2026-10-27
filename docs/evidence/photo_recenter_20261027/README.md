# 2026-10-27 拍照后未回中

任务：`data/captures/missions/rviz_patrol_20261027_093835_20261027_093835`。

北京时间 09:39:35，P002 导航成功；09:39:38～09:39:44，left/center/right 三张照片全部成功，最后记录 heading=28.06°。随后回中前的一次停车检查读到 twist=0.032 m/s，超过 0.03 m/s 门限，直接触发失败。没有 gimbal_recenter_started 事件，意味着回中动作根本没发送。09:39:47 终止清理又确认 base_stationary=true。

任务最终 failed，error_code=1100，completed_points=1，capture_records=6。P003 和 RETURN_HOME 未执行。RViz 中仍能看到路径并不意味着父任务仍在导航。不能仅凭这次 0.032 m/s 判断是实际底盘移动还是定位速度波动。

修复：回中前复用 wait_until_stationary，而非单帧检查。默认最多等待 8 秒，需连续 0.8 秒满足原有速度、位姿和新鲜度门限；取消会中断等待，超时仍失败，不放宽停车要求。等待期间发布 STABILIZING 状态，回中反馈成功后才允许下一点导航。未主动重启当前设备或发送云台/底盘动作。

隔离 ROS_DOMAIN_ID=183、ROS_LOCALHOST_ONLY=1 回归包含回中成功、失败、错误反馈角、动作超时、动作取消，以及新增瞬时运动恢复、等待停车超时、等待时取消。
