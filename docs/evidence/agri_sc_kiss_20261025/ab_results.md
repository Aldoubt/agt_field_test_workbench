# 合成固定地图 A/B 实测

| 指标 | BBS + GICP（无候选整图回退） | SC + KISS + GICP |
|---|---|---|
| 是否成功 | False | True |
| 粗定位耗时 | NOT_REPORTED | 0.116493 s |
| 精配准耗时 | NOT_REPORTED_BY_EXISTING_EXECUTABLE | 0.065073 s |
| 总耗时 | 30.071538 s | 0.475451 s |
| Fitness | NOT_RUN | 0.00218843 |
| 位置误差 | NOT_RUN | 0.000058856 m (SYNTHETIC_KNOWN_SE3) |
| 航向误差 | NOT_RUN | 0.000004075 rad (SYNTHETIC_KNOWN_SE3) |
| 是否错误接受 | NOT_RUN | False |
| CPU user/system | 59.15/0.44 s | 0.55/0.19 s |
| 峰值 RSS | 73464 KiB | 99528 KiB |

单个合成查询、非农科院数据；BBS 30 秒超时。SC 检索出 000001.pcd，KISS 最终 682 内点。
coarse/GICP 时间为最终选中候选的阶段耗时；总耗时包含所有候选、输入处理及进程开销。
RSS 是 GNU time 单进程最大驻留集，含等待子进程的最大值，并非同时运行进程树内存之和。
REAL_BAG_VERIFIED=NOT_RUN；HARDWARE_VERIFIED=NOT_RUN。
