# IMU Params {#spec_imu_params}

## IMU Intrinsics

| Name | Field | Unit | Bytes | Note |
| :----- | :----- | :----- | :-------- | :----- |
| Scale matrix | acc_scale[3][3] | - | 72 | double |
| ^ | gyro_scale[3][3] | - | 72 | double |
| Zero-drift | acc_drift[3] | - | 24 | double |
| ^ | gyro_drift[3] | - | 24 | double |
| Noise density | acc_noise[3] | - | 24 | double |
| ^ | gyro_noise[3] | - | 24 | double |
| Random walk | acc_bias[3] | - | 24 | double |
| ^ | gyro_bias[3] | - | 24 | double |

## IMU Extrinsics

Transformation matrix from left image to IMU.

| Name | Field | Unit | Bytes | Note |
| :----- | :----- | :----- | :-------- | :----- |
| Rotation matrix | rotation[3][3] | - | 72 | double |
| Translation vector | translation[3] | - | 24 | double |
