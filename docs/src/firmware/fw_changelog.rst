Change Log
==================

.. toctree::
   :maxdepth: 2

2019-11-25
-------------------

S1030 Main Chip Firmware: MYNTEYE-S1030-2.7.0.img

1. Fix some issues

S21X0 Main Chip Firmware: MYNTEYE-S21X0-2.1.0.img

1. Update protocol version
2. Fix and optimize some other issues

2019-09-09
-------------------

S21X0 Main Chip Firmware: MYNTEYE-S21X0-1.4.0.img

1. Support 2110 device.

2019-08-09
-------------------

S1030 Main Chip Firmware: MYNTEYE-S1030-2.5.0.img

1. Optimize the synchronization of image and imu
2. Not save the camera control parameters
3. Fix the overexplosion problem at low resolution
4. Fix USB 2.0 first open failure problem
5. Add automatic recovery function when updating wrong firmware

S2100 Main Chip Firmware: MYNTEYE-S2100-1.3.2.img

1. Optimize the synchronization of image and imu
2. Not save the camera control parameters
3. Optimize IMU low-pass filter default values
4. Optimize the exposure time calculation method, the maximum exposure time is limited to 66.5ms
5. Add automatic recovery function when updating wrong firmware
6. Fix and optimize some other issues

S2100 Auxiliary Chip Firmware: MYNTEYE-S2100-auxiliary-chip-1.4.2.bin

1. Time synchronization adds uart interface, io interruption judgement
2. Time synchronization i2c interface adds whoami, read timestamp and median filter open state interface
3. Fix and optimize some other issues
