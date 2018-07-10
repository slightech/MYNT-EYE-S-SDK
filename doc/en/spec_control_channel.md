# Control Channels {#spec_control_channel}

| Name | Field | Address | Bandwidth | Node |
| :----- | :----- | :----- | :----- | :----- |
| Camera control channel | XU_CAM_CTRL_CHANNEL | 1 | 3 | |
| Half-Duplex channel | XU_HALF_DUPLEX_CHANNEL | 2 | 20 | |
| IMU write channel | XU_IMUDATA_WRITE_CHANNEL | 3 | 5 | |
| IMU read channel | XU_IMUDATA_READ_CHANNEL | 4 | 2000 | |
| File channel | XU_FILE_CHANNEL | 5 | 2000 | |

## Camera Control Channel

The channel provides get, set and query (min, max, default).

## Half-Duplex Channel

The channel only provides set, such as zero drift correction.

## IMU Channel

The channel is used to request and response IMU data, see @ref spec_imu_data.

## File Channel

The channel is used to read and write device information, image params, and IMU params.

| Name | Header | Size | File | Checksum |
| :--- | :- | :--- | :--- | :-------- |
| Bytes | 1 | 2 | - | 1 |
| Type | uint8_t | uint16_t | - | uint8_t |
| Description | Flags | Content size | Content data | Checksum, XOR of all content bytes |

| Header Bit Subscript | Description |
| :------------------- | :---------- |
| 0 | Device information |
| 1 | Image params |
| 2 | IMU params |
| 3~6 | Undefined |
| 7 | 0: Get; 1: Set |

### File Content Packet

| Name | ID | Size | Content |
| :--- | :- | :--- | :------ |
| Bytes | 1 | 2 | - |
| Type | uint8_t | uint16_t | - |
| Description | Content ID | Content size | Content data |

| File | ID | Max Size |
| :--- | :- | :------- |
| Device information | 1 | 250 |
| Image params | 2 | 250 |
| IMU params | 4 | 500 |
