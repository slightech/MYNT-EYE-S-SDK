# Image Params {#spec_image_params}

## Image Intrinsics

| Name | Field | Unit | Bytes | Note |
| :----- | :----- | :----- | :-------- | :----- |
| Image width | width | px | 2 | uint16_t; [0,65535] |
| Image height | height | px | 2 | uint16_t; [0,65535] |
| Focal length | fx | - | 8 | double |
| ^ | fy | - | 8 | double |
| Principal point | cx | - | 8 | double |
| ^ | cy | - | 8 | double |
| Distortion model | model | - | 1 | uint8_t; pinhole,... |
| Distortion coefficients | coeffs[5] | - | 40 | double; k1,k2,p1,p2,k3 |

## Image Extrinsics

Transformation matrix from left image to right image.

| Name | Field | Unit | Bytes | Note |
| :----- | :----- | :----- | :-------- | :----- |
| Rotation matrix | rotation[3][3] | - | 72 | double |
| Translation vector | translation[3] | - | 24 | double |
