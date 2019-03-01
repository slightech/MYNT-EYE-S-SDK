# Device Information {#spec_hardware_info}

| Name | Field | Fixed Value | USB Descriptor | UVC Extension Unit | Bytes | Note |
| :----- | :----- | :-------- | :-------------- | :----------------- | :-------- | :----- |
| VID | vid | 0x04B4 | √ | × | 2 | |
| PID | pid | 0x00F9 | √ | × | 2 | |
| Device name | name | MYNT-EYE-? | √ | √ Get | 16 | MYNT-EYE-S1000 |
| Serial number | serial_number | - | √ | √ Get | 16 | |
| Firmware version | firmware_version | - | √ | √ Get | 2 | major,minor |
| Hardware version | hardware_version | - | × | √ Get | 3 | major,minor,flag |
| Spec version | spec_version | - | × | √ Get | 2 | major,minor |
| Lens type | lens_type | - | × | √ Get/Set | 4 | vendor(2),product(2); default: 0 |
| IMU type | imu_type | - | × | √ Get/Set | 4 | vendor(2),product(2); default: 0 |
| Nominal baseline | nominal_baseline | - | × | √ Get/Set | 2 | unit: mm; default: 0 |
| Auxiliary chip version | auxiliary_chip_version | - | × | √ Get | 2 | major,minor |
| isp version | isp_version | - | × | √ Get | 2 | major,minor |
