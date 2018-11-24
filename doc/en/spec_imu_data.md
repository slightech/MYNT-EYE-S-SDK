# IMU Data {#spec_imu_data}

## IMU Request Packet

| Name | Header | Serial Number |
| :--- | :----- | :------------ |
| Bytes | 1 | 4 |
| Type | uint8_t | uint32_t |
| Description | 0x5A | First request should be 0, otherwise the last one |

## IMU Response Packet

The IMU response packet contains multiple IMU packets, and each IMU packet contains multiple IMU segments.

| Name | Header | State | Size | IMU Packets | Checksum |
| :--- | :----- | :---- | :--- | :---------- | :------- |
| Bytes | 1 | 1 | 2 | ... | 1 |
| Type | uint8_t | uint8_t | uint16_t | - | uint8_t |
| Description | 0x5B | 0 is success, others are failed | Content size | IMU packets | Checksum, XOR of all content bytes |

### IMU Packet

The IMU packet is an array of IMU datas.

| Name | Serial Number | Timestamp | Count | IMU Datas |
| :--- | :------------ | :-------- | :---- | :-------- |
| Bytes | 4 | 4 | 1 | ... |
| Type | uint32_t | uint32_t | uint8_t | - |
| Description | Serial number | IMU basic timestamp | The number of IMU datas | IMU datas |

### IMU Segment

| Name | Offset | Frame ID | Accelerometer | Temperature | Gyroscope |
| :--- | :----- | :------- | :------------ | :---------- | :-------- |
| Bytes | 2 | 2 | 6 | 2 | 6 |
| Type | int16_t | uint16_t | int16_t * 3 | int16_t | int16_t * 3 |
| Description | The timestamp offset | Image frame ID | Accel x,y,z values | IMU temperature | Gyro x,y,z values |

* Formula for converting the accel & gyro values to real ones: **real = data * range / 0x10000** .
  * ``accel`` default ``range`` is **8 g**, ``gyro`` default ``range`` is **1000 deg/s**.
* Formula for converting the temperature to real value: **real = data / ratio + offset** .
  * default ``ratio`` is **326.8**, default ``offset`` is **25℃**.
