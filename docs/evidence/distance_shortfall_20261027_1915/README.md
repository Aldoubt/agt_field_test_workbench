# 2026-10-27 19:15 5m测试短行程与弯曲路径

操作人员反馈：向前5m经常偏短约18cm，横向控制表现尚可；截图显示规划路径有弯曲、任务canceled。这里保留操作者测量反馈，没有同步独立测距数据校验。

本轮目标由 start_pose=(19.3376341709,-9.2028339235)、yaw=-0.1112711401 计算得到 (24.3067129278,-9.7580422697)，XY距离=5.000000000000m，目标生成没有减去18cm。

## 明确发现

会话 runs/field_trial_20261027T111434Z_8f7f2227/session.json 的 planning_profile=navigation；日志加载 nav2_smac_planner/SmacPlanner2D。当前代码的普通导航 general_goal_checker.xy_goal_tolerance=0.08，distance才使用0.02、ExactGoalPlanner与w_data=0的距离平滑设置。本次没有启用距离专用配置。

19:15:19开始导航；19:15:51、19:16:07、19:16:22先后Failed to make progress，19:16:22.98取消。任务记录 completed_points=0、canceled，不能当作2cm精度正常到点结果。18cm超过普通8cm半径，不能声称仅改容差就解释或修复全部误差。

本轮没有对应的新bag或留存的逐帧/plan与底盘速度证据，无法量化路径末端误差、低速死区或LIO漂移各自贡献。路径弯曲可能来自栅格搜索/平滑与障碍代价；现有距离模式已针对格点弯曲调整，但不能保证有障碍时仍走直线。

## 本次修改

field_distance run/home在提交Action之前读取实际controller_server与planner_server参数，要求位置容差不大于2cm、ExactGoalPlanner与距离平滑参数。普通navigation配置、错误规划器或错误平滑配置会明确报错并提示 ./field_start distance，不再因camera-free runtime相同而误认为已启用精度配置。

未通过改目标距离补偿18cm，也未绕过碰撞检查。2cm是定位坐标中的判定半径，不是物理测距精度保证；实车停车需复测。

## 复测入口

停车后在旧会话终端Ctrl+C，终端A执行 ./field_start distance，等待就绪。终端B执行 ./field_trial record；终端C执行 ./field_distance run 5。每次run取当时位置和朝向，不复用旧HOME_START。结束后停止录包并保留包。若临近终点再次停滞，保存停止瞬间与任务终态，分别比较：目标与/plan末端、底盘base_link到目标的纵横残差、/cmd_vel_nav与/mux/cmd_vel、/wheel/odom与/agt/odometry/local、实时障碍及代价地图。

私有DDS域183/localhost，无物理控制器：距离提交参数服务检查、任务生成、原生规划器回归共30项通过。规划器回归验证精确终点、自由空间平滑以及障碍/未知区域避让；不代表完成实车停车验收。
