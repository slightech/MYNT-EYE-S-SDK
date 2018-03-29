# IMU 参数说明 {#spec_imu_params}

## IMU 内参

| 名称 | 字段 | 单位 | 字节数 | 说明 |
| :----- | :----- | :----- | :-------- | :----- |
| 比例因子 | acc_scale[3][3] | - | 72 | double |
| ^ | gyro_scale[3][3] | - | 72 | double |
| 零漂 | acc_drift[3] | - | 24 | double |
| ^ | gyro_drift[3] | - | 24 | double |
| 噪声密度 | acc_noise[3] | - | 24 | double |
| ^ | gyro_noise[3] | - | 24 | double |
| 随机游走 | acc_bias[3] | - | 24 | double |
| ^ | gyro_bias[3] | - | 24 | double |

## IMU 外参

Left Image 到 IMU 的变换矩阵。

| 名称 | 字段 | 单位 | 字节数 | 说明 |
| :----- | :----- | :----- | :-------- | :----- |
| 旋转矩阵 | rotation[3][3] | - | 72 | double |
| 平移矩阵 | translation[3] | - | 24 | double |
