# 图像参数说明 {#spec_image_params}

## 图像内参

| 名称 | 字段 | 单位 | 字节数 | 说明 |
| :----- | :----- | :----- | :-------- | :----- |
| 宽度 | width | px | 2 | uint16_t; [0,65535] |
| 高度 | height | px | 2 | uint16_t; [0,65535] |
| 焦距 | fx | - | 8 | double |
| ^ | fy | - | 8 | double |
| 图像中心 | cx | - | 8 | double |
| ^ | cy | - | 8 | double |
| 畸变模型 | model | - | 1 | uint8_t; pinhole,... |
| 畸变参数 | coeffs[5] | - | 40 | double; k1,k2,p1,p2,k3 |

> 图像分辨率不同，内参不同。多分辨率的话，需有多个内参。

## 图像外参

Left Image 到 Right Image 的变换矩阵。

| 名称 | 字段 | 单位 | 字节数 | 说明 |
| :----- | :----- | :----- | :-------- | :----- |
| 旋转矩阵 | rotation[3][3] | - | 72 | double |
| 平移矩阵 | translation[3] | - | 24 | double |
