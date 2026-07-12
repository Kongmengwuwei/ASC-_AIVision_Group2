# VehicleAutoCalibration

该工具通过 HC-04 蓝牙向车辆下发测试动作，同时读取 `SmartCarPoseMonitor` 的真实
`X/Y/Yaw`，自动计算并验证控制参数。上位机位置只在电脑端用于测量，不发送给车端。

## 安全工作流

1. 连接 DAP，烧录带运行时标定接口的固件。
2. 断电并拔掉 DAP 数据线。
3. 将车放在地图中央开阔区域，四个方向至少保留 0.8 m。
4. 给车上电，连接 HC-04 蓝牙串口。
5. 启动 `SmartCarPoseMonitor` 并确认位姿持续更新。
6. 双击 `Start-Calibration.cmd`，选择蓝牙 COM 口并输入 `RUN`。

任何时刻按 `Ctrl+C` 都会发送 `stop`。位姿超过 700 ms 未更新、车辆越过地图安全边界、
位移或旋转超过测试包络时，程序也会自动停车并终止。

## 分阶段运行

默认先运行 `kinematics`，辨识：

- 前后线性尺度 `kin.linear`
- 横移尺度 `kin.lateral`
- 横移—纵向耦合 `kin.coupling`

其余阶段可从 PowerShell 运行：

```powershell
.\Start-Calibration.ps1 -Phase yaw -Port COM7
.\Start-Calibration.ps1 -Phase position -Port COM7
.\Start-Calibration.ps1 -Phase guide -Port COM7
.\Start-Calibration.ps1 -Phase scurve -Port COM7
.\Start-Calibration.ps1 -Phase all -Port COM7
```

为了安全和便于检查，建议第一次不要直接运行 `all`。每一阶段完成后，轨迹 CSV 和
`recommended_parameters.txt` 会保存在 `results/时间戳/`。

## 调参内容

- `yaw`：Yaw Kp、Kd、最小角速度前馈
- `position`：近目标位置环 Kp、Kd
- `guide`：法向纠偏 Kp（同时评估是否关闭反而更好）
- `scurve`：加速度和加加速度，在精度约束下选择较快组合

最终推荐值需要固化回源码并再次烧录验证；运行时标定值断电后不会保存。
