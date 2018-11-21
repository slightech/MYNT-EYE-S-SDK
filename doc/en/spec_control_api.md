# Control Protocols {#spec_control_api}

There are two control modes, one is through UVC standard protocol, the other is through UVC custom protocol with extension unit.

## Standard Protocol

| Name | Field | Bytes | Default | Min | Max | Stored | Flash Address | Note |
| :--- | :---- | :---- | :------ | :-- | :-- | :----- | :------------ | :--- |
| Gain | gain | 2 | 24 | 0 | 48 | √ | 0x12 | valid if manual-exposure |
| Brightness | brightness/exposure_time | 2 | 120 | 0 | 240 | √ | 0x14 | valid if manual-exposure |
| Contrast | contrast/black_level_calibration | 2 | 127 | 0 | 255 | √ | 0x10 | valid if manual-exposure |

## Custom Protocol

| Name | Field | Bytes | Default | Min | Max | Stored | Flash Address | Channel | Note |
| :--- | :---- | :---- | :------ | :-- | :-- | :----- | :------------ | :------ | :----- |
| Frame rate | frame_rate | 2 | 25 | 10 | 60 | √ | 0x21 | XU_CAM_CTRL | values: {10,15,20,25,30,35,40,45,50,55,60} |
| IMU frequency | imu_frequency | 2 | 200 | 100 | 500 | √ | 0x23 | XU_CAM_CTRL | values: {100,200,250,333,500} |
| Exposure mode | exposure_mode | 1 | 0 | 0 | 1 | √ | 0x0F | XU_CAM_CTRL | 0: enable auto-exposure; 1: manual-exposure |
| Max gain | max_gain | 2 | 48 | 0 | 48 | √ | 0x1D | XU_CAM_CTRL | valid if auto-exposure |
| Max exposure time | max_exposure_time | 2 | 240 | 0 | 240 | √ | 0x1B | XU_CAM_CTRL | valid if auto-exposure |
| Desired brightness | desired_brightness | 2 | 192 | 0 | 255 | √ | 0x19 | XU_CAM_CTRL | valid if auto-exposure |
| IR control | ir_control | 1 | 0 | 0 | 160 | × | - | XU_CAM_CTRL | |
| HDR mode | hdr_mode | 1 | 0 | 0 | 1 | √ | 0x1F | XU_CAM_CTRL | 0: 10-bit; 1: 12-bit |
| Zero drift calibration | zero_drift_calibration | | - | - | - | × | - | XU_HALF_DUPLEX | |
| Erase chip | erase_chip | | - | - | - | × | - | XU_HALF_DUPLEX | |
