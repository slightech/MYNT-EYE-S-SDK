# 拓展通道说明 {#spec_contorl_channel}

| 名称 | 字段 | 地址 | 带宽 | 说明 |
| :----- | :----- | :----- | :----- | :----- |
| 相机控制通道 | XU_CAM_CTRL_CHANNEL | 0x0100 | 3 | |
| 半双工通道 | XU_HALF_DUPLEX_CHANNEL | 0x0200 | 20 | |
| IMU 请求通道 | XU_IMUDATA_WRITE_CHANNEL | 0x0300 | 5 | |
| IMU 响应通道 | XU_IMUDATA_READ_CHANNEL | 0x0400 | 2000 | |
| 文件通道 | XU_FILE_CHANNEL | 0x0500 | 2000 | |

## 相机控制通道

相机控制通道是那些需要 Get & Set & Query 的控制通道，其中 Query 细分为 Min, Max, Default 。

## 半双工通道

半双工通道是那些仅需 Get 或 Set 的控制通道，如请求零漂矫正。

## IMU 通道

用来请求和响应 IMU 数据的通道，可参见 @ref spec_imu_data 。

## 文件通道

用来读写硬件信息、图像参数、 IMU 参数的通道。

| Name | Flag | Size | File | Checksum |
| :--- | :- | :--- | :--- | :------- |
| 字节数 | 1 | 2 | - | 1 |
| 类型 | uint8_t | uint16_t | - | uint8_t |
| 描述 | 标识 | 数据包大小 | 文件内容 | 校验码（Flag 以外所有包字节异或） |

| File | Flag 位 | ID |
| :----- | :------- | :- |
| 硬件信息 | 0 | 0 |
| 图像参数 | 1 | 1 |
| IMU 参数 | 2 | 2 |

### 文件内容包

| Name | ID | Size | Content |
| :--- | :- | :--- | :------ |
| 字节数 | 1 | 2 | - |
| 类型 | uint8_t | uint16_t | - |
| 描述 | 内容 ID | 内容包大小 | 内容 |
