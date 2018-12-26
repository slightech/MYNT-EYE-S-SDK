# IMU 数据说明 {#spec_imu_data}

## IMU 请求数据包

| Name | Header | Serial Number |
| :--- | :----- | :------------ |
| 字节数 | 1 | 4 |
| 类型 | uint8_t | uint32_t |
| 描述 | 0x5A | 首次请求写 0 ，不然写上次响应数据包最后一个 IMU 包的序列号 |

## IMU 响应数据包

IMU 响应数据包里会包含1个 IMU 包，而每个 IMU 包又带有多个 IMU 段。

| Name | Header | State | Size | IMU Packets | Checksum |
| :--- | :----- | :---- | :--- | :---------- | :------- |
| 字节数 | 1 | 1 | 2 | ... | 1 |
| 类型 | uint8_t | uint8_t | uint16_t | - | uint8_t |
| 描述 | 0x5B | 正常状态为 0 ，否则错误 | 数据内容大小 | 所包含的 IMU 包 | 校验码（数据内容所有字节异或） |

### IMU 包

IMU 包/小包，是一组 IMU 数据。

| Name | Count | IMU Datas |
| :--- | :-----| :-------- |
| 字节数 | 2 | ... |
| 类型 | uint16_t | - |
| 描述 | IMU 段数量 | 所包含的 IMU 段 |

### IMU 段

| Name | Serial Number | Timestamp | flag |  Temperature | Accelerometer or Gyroscope |
| :--- | :------------ | :-------- | :----| :----------- | :------------------------- |
| 字节数 | 4 | 8 | 1 | 2 | 6 |
| 类型 | uint32_t | uint64_t | int8_t | int16_t | int16_t * 3 |
| Description | 序列号 | 时间戳 | 指定传感器类型 | IMU 的温度 | 陀螺仪或陀螺仪 x y z 三轴的值 |

* 加速度计和陀螺仪的计量值换算成物理值公式： **real = data * range / 0x10000** 。
  * 加速度计量程默认值为 **12 g** ，陀螺仪量程默认值为 **1000 deg/s** 。
* 温度计量值换算成物理值公式： **real = data / ratio + offset** 。
  * ``ratio`` 默认值为 **326.8** ， ``offset`` 默认值为 **25℃** 。
