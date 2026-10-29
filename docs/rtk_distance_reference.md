# 独立RTK观测与5m测试参考

2026-10-27实际接入验证：ASENSING兼容二进制串口输出，设备具体型号在仓库中未记载；双天线安装于车上独立支架来自操作人员描述。USB转串口新设备为Prolific适配器，对应/dev/ttyUSB1；优先使用下面稳定by-id路径。

## 启动

在工作区根目录运行：

```bash
bash src/agt_field_test_workbench/scripts/ros_exec.sh \
  ros2 run agt_asensing_driver r3_drive_validation.sh \
  --port /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DTAZj137C01-if00-port0 \
  --baudrate 460800 --label rtk_distance --static-window 60 \
  --output-root "$PWD/work/rtk_trials"
```

此命令启动INS驱动、质量监测和独立录包，不发送运动命令；Ctrl-C后结束并保存。保持初始窗口内车辆停稳。运行目录包含report.md、report.json、driver.log、monitor.log和bag/。输出报告标签EVIDENCE_ONLY，不自动认定高精度。不要与其他驱动争用串口；已由其他程序提供INS话题时用--no-driver。需要和导航录包同步时使用同一ROS域并记录/ins/navsatfix和/ins/status，GNSS观测不接入map->odom。

## 实测与限制

原驱动最高支持230400，本机设备实际是460800：3s原始采样校验通过297帧；已新增460800支持、启动参数，并修复监测窗口传入整数时的ROS double参数类型错误。工作台依赖固定到INS提交20e804c951ff603fc87bc1a77da36462952e4b0a，该SHA是本次可复现驱动版本，不表示完整R3硬件验收或厘米定位已通过。

约30s独立观测：2848条NavSatFix、2847条INSStatus，有协方差的有效fix接收约99.96Hz，原始position_type=16、heading_type=50、末次卫星24颗，当前rtk_fixed_types=[4]归一化结果固定解计数0。原始枚举含义需设备手册或供应商确认，未擅自把16/50改成固定位置解。

设备自报E/N/U标准差约0.383/0.375/0.533m。窗口内位置散布比这些值小，但既未独立确认停车，也没有外部真值；不能把短时稳定或多采样平均直接宣称为厘米准确度。当前NOT_READY_AS_18CM_REFERENCE。双天线朝向解与位置固定解分别判断，不因heading_type=50而认定位置已固定。

本地证据：work/rtk_probe_20261027，完整原始经纬度/二进制/ROSbag仅保存在本机；云端只保存不含绝对坐标的质量汇总。

## 如何作为独立距离参考

1. 核实设备差分改正来源及接收状态，核对position_type固定解枚举，在开阔位置保持静止至少60s并观察固定状态、E/N标准差和漂移。自报精度与实际准确度分别记录。当前约38cm自报水平标准差不宜用来裁定18cm偏差。
2. 在起点和终点分别停车记录稳定窗口，将两端经纬度转换到共同局部ENU，比较端点均值的水平位移：sqrt(deltaE²+deltaN²)。不要累计逐帧抖动当作行程，也不要把100Hz相关样本数量当作独立精度提升。
3. 明确测量点是接收机输出参考点/主天线点，核实厂内杆臂设置；它不自动等于base_link。固定安装且两端朝向一致时，恒定杆臂对端点位移可相消。朝向变化时应测量base_link到参考点的杆臂，并在验证姿态约定后再补偿。例如1m偏移随10度旋转会产生约17cm的点位变化，这只是几何示例，不是本车已测杆臂。
4. 驱动纬经度协议编码分辨率为1e-7度，纬向约1.1cm；不要输出毫米级精度结论。/ins/odom没有已接受的局部笛卡尔位置语义，不能直接作为测距真值；/ins/pose姿态约定也尚待物理验证。
5. 参考质量合格后，再与Nav2目标、LIO位移、轮速位移和物理卷尺测量对照。RTK在此作为独立观测，不改导航定位来源。

完整驱动记录位于src/agt_ins_driver/docs/rtk-gnss-contract/records/R03_20261027_460800_FIELD_PROBE.md。该次17项软件测试通过、独立启动/报告/录包和退出流程实测完成；全套固定解、方向、杆臂和实车精度验收仍待执行。
