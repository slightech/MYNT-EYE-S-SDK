# 硬件信息说明 {#spec_hardware_info}

| 名称 | 字段 | 固定值 | 描述符获取 | 拓展通道获取 | 字节数 | 说明 |
| :----- | :----- | :-------- | :-------------- | :----------------- | :-------- | :----- |
| VID | vid | 0x04B4 | √ | × | 2 | |
| PID | pid | 0x00F9 | √ | × | 2 | |
| 设备名称 | name | MYNT-EYE-? | √ | √ Get | 16 | MYNT-EYE-S1000 |
| 序列号  | serial_number | - | √ | √ Get | 16 | |
| 固件版本 | firmware_version | - | √ | √ Get | 2 | major,minor |
| 硬件版本 | hardware_version | - | × | √ Get | 3 | major,minor,flag |
| 协议版本 | spec_version | - | × | √ Get | 2 | major,minor |
| 镜头类型 | lens_type | - | × | √ Get/Set | 4 | vendor(2),product(2) ，未 Set 默认 0 |
| IMU 类型 | imu_type | - | × | √ Get/Set | 4 | vendor(2),product(2) ，未 Set 默认 0 |
| 基线长度 | nominal_baseline | - | × | √ Get/Set | 2 | 单位 mm ，未 set 默认 0 |

* 描述符获取：指通用 USB 设备信息，可用工具查看。
* 拓展通道获取：指通过拓展通道（UVC Extension Unit）问硬件获取到的信息，需要读取。
