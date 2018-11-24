# Image Data {#spec_image_data}

| Name | Field | Unit | Bytes | Note |
| :----- | :----- | :----- | :-------- | :----- |
| Frame ID | frame_id | - | 2 | uint16_t; [0,65535] |
| Timestamp | timestamp | 10 us | 4 | uint32_t |
| Exposure Time | exposure_time | 10 us | 2 | uint16_t |

## Image Packet

| Name | Header | Size | Frame ID | Timestamp | Exposure Time | Checksum |
| :--- | :----- | :--- | :------- | :-------- | :------------ | :------- |
| Bytes | 1 | 1 | 2 | 4 | 2 | 1 |
| Type | uint8_t | uint8_t | uint16_t | uint32_t | uint16_t | uint8_t |
| Description | 0x3B | 0x08, content size | Frame ID | Timestamp | Exposure time | Checksum, XOR of all content bytes |

* The image packet will be dropped, if checksum is incorrect.
* The accuracy of the time unit: 0.01 ms / 10 us.
  * The timestamp could indicate 11.9 hours, it will accumulate again after overflow.
* The timestamp accumulation starts from the time of power-on, instead of opening.
