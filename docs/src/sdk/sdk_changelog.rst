.. _sdk_changelog:

Change Log
==========
2019-10-21(v2.5.0)
-------------------

1. Change project directory structure.
2. Lifting dependency and improving code quality.
3. Support xy demo code in sample get_depth_with_region.
4. Fix some bugs.

2019-09-4(v2.4.2)
-------------------

1. Part of the sample examples were sorted out, and useless tools and engineering code were removed.
2. Optimize disparity calculation.
3. Improving the logic of model display.
4. Fix some bugs.

2019-08-17(v2.4.1)
-------------------

1. Optimize disparity calculation

2019-08-09(v2.4.0)
-------------------

1. Optimize the synchronization of images and imu
2. Add 4.16+ kernel support on Ubuntu
3. Fix missinf frame_id issue in image information
4. Fix S1030 device not work issue in mynteye_multiple.launch
5. Add save single picture sample save_single_image

2019-07-03(v2.3.9)
-------------------

1. Fix ros timestamp issue
2. Add calibration tool doc

2019-05-20(v2.3.8)
-------------------

1. Improve VINS-Fusion supporting
2. Improve VINS-MONO supporting
3. Fix left/right rect image order error

2019-04-19(v2.3.7)
-------------------

1. Improve VINS-Fusion supporting
2. Improve ORB-SLAM2 supporting

2019-04-15(v2.3.6)
-------------------

1. Fix imu align bug of ros wrapper
2. Fix 14.04 complie error of ros wrapper
3. Support set iic address for s2100

2019-04-01(v2.3.5)
-------------------

1. Improve camera info
2. Modify image algorithm parameters by yaml file
3. Add opening multi devices launch file in ROS
4. Add setting IIC address API of S210A
5. Add image/imu flag of external time source
6. Add LaserScan sample for S1030
7. Modify default orientation of point in ROS

2019-03-18(v2.3.4)
-------------------

1. Add API to get auxiliary chip&ISP's version (Depend on S2100/S210A 1.1 firmware & 1.0 subsidiary chip firmware)
2. Fix point fragment issue in image algorithm
3. Add 376*240 resolution support to S1030 (Depend on 2.4.0 firmware of S1030)
4. Add API to handle imu temperature drift (Depend on imu calibration)
5. Add version check feature
6. Fix depth image crash issue when use CUDA plugin
7. Documents update
