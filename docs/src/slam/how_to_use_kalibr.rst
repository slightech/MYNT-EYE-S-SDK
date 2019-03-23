.. _how_to_use_kalibr:

How to calibrate MYNTEYE by kalibr
===================================

Target
------------

* Calibrate the pose relationship between left and right camera

* Calibrate the pose relationship left camera between and IMU


Preparation
------------

* **Install kalibr**:Refer to `kalibr wiki <https://github.com/ethz-asl/kalibr/wiki/installation>`_ and follow the steps to install

* **Calibration board**： kalibr supports ``chessbord`` , ``circlegrid`` , ``aprilgrid`` ,choose ``aprilgrid`` here Calibration board file can be directly `download <https://github.com/ethz-asl/kalibr/wiki/downloads>`_ , Or you can also generate calibration board by Kalibr tool.

.. code-block:: bash

  $ kalibr_create_target_pdf --type 'apriltag' --nx 6 --ny 6 --tsize 0.08 --tspace 0.3

View parameters' meanings by kalibr_create)target_pdf command:

.. code-block:: bash

  $ kalibr_create_target_pdf --h
  usage:
      Example Aprilgrid:
          kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.08 --tspace 0.3
      Example Checkerboard:
          kalibr_create_target_pdf --type checkerboard --nx 6 --ny 6 -csx 0.05 --csy 0.1


  Generate PDFs of calibration patterns.

  optional arguments:
    -h, --help           show this help message and exit

  Output options:
    output               Output filename
    --eps                Also output an EPS file

  Generic grid options:
    --type GRIDTYPE      The grid pattern type. ('apriltag' or 'checkerboard')
    --nx N_COLS          The number of tags in x direction (default: 6)
    --ny N_ROWS          The number of tags in y direction (default: 7)

  Apriltag arguments:
    --tsize TSIZE        The size of one tag [m] (default: 0.08)
    --tspace TAGSPACING  The space between the tags in fraction of the edge size
                        [0..1] (default: 0.3)
    --tfam TAGFAMILIY    Familiy of April tags ['t16h5', 't25h9', 't25h7',
                        't36h11'] (default: t36h11)

  Checkerboard arguments:
    --csx CHESSSZX       The size of one chessboard square in x direction [m]
                        (default: 0.05)
    --csy CHESSSZY       The size of one chessboard square in y direction [m]
                        (default: 0.05)

* **Calibrate the intrinsic IMU parameters** ：kalibr requires imu data to be calibrated by intrinsic parameters by default.The intrinsic parameters calibration tool uses `imu-tk <https://github.com/Kyle-ak/imu_tk.git>`_ .

* **Count imu data parameter**：
    * noise density
    * bias random walk

Using Allan analyzing tool `imu_utils <https://github.com/gaowenliang/imu_utils>`_, We can get the characteristics of above imu data,and to format the output as ``imu.yaml``

.. code-block:: bash

  #Accelerometers
  accelerometer_noise_density: 0.02680146180736048   #Noise density (continuous-time)
  accelerometer_random_walk:   0.0026296086159332804   #Bias random walk

  #Gyroscopes
  gyroscope_noise_density:     0.008882328296710996   #Noise density (continuous-time)
  gyroscope_random_walk:       0.00037956578292701033   #Bias random walk

  rostopic:                    /mynteye/imu/data_raw      #the IMU ROS topic
  update_rate:                 200.0      #Hz (for discretization of the values above)

Calibrate the pose relationship between left and right camera
--------------------------------------------------------------

* Collect calibration images: kalibr supports the collection of the required calibration images through two ways:by ``rosbag`` or collect offline images . Using ``rosbag`` here for convenience,Reference `link <https://github.com/ethz-asl/kalibr/wiki/bag-format>`_ for collecting images.
* Method of collecting images by ``rosbag`` :fix mynteye camera,move ``aprilgrid`` calibration board in the camera field of view.
* To increase the calibration time,try to use image acquisition data with lower frame rate,kalibr recommends using ``4Hz`` frame rate,here uses ``10hz`` .
* MYNTEYE S series camera offers images at least 10Hz,You can use `topic_tools <http://wiki.ros.org/topic_tools/throttle>`_ to modify frequency,because using 10Hz requires more calibration time.
* Record ``static.bag`` : After fix the mynteye camera,start `wrapper <https://github.com/slightech/MYNT-EYE-S-SDK>`_, record the topic of the left and right images to ``static.bag`` .

.. code-block:: bash

  $ source wrappers/ros/devel/setup.bash
  $ roslaunch mynt_eye_ros_wrapper display.launch
  $ cd ~
  $ mkdir -p bag
  $ cd bag
  $ rosbag record -O static_10hz /mynteye/left/image_raw /mynteye/right/image_raw #recommand use 10hz,you can also use topic_tools to publish 4hz.

* kalibr calibration:

.. code-block:: bash

  $ kalibr_calibrate_cameras --target aprilgrid.yaml --bag ~/bag/static_10hz.bag --models pinhole-radtan pinhole-radtan --topics /mynteye/left/image_raw /mynteye/right/image_raw

View parameters' meanings by kalibr_calibrate_cameras command:

.. code-block:: bash

  $ kalibr_calibrate_cameras --h

  Calibrate the intrinsics and extrinsics of a camera system with non-shared
  overlapping field of view.

  usage:
    Example usage to calibrate a camera system with two cameras using an aprilgrid.

    cam0: omnidirection model with radial-tangential distortion
    cam1: pinhole model with equidistant distortion

    kalibr_calibrate_cameras --models omni-radtan pinhole-equi --target aprilgrid.yaml \
              --bag MYROSBAG.bag --topics /cam0/image_raw /cam1/image_raw

    example aprilgrid.yaml:
        target_type: 'aprilgrid'
        tagCols: 6
        tagRows: 6
        tagSize: 0.088  #m
        tagSpacing: 0.3 #percent of tagSize

  optional arguments:
  -h, --help            show this help message and exit
  --models MODELS [MODELS ...]
                        The camera model ['pinhole-radtan', 'pinhole-equi',
                        'omni-radtan', 'pinhole-fov'] to estimate

  Data source:
  --bag BAGFILE         The bag file with the data
  --topics TOPICS [TOPICS ...]
                        The list of image topics
  --bag-from-to bag_from_to bag_from_to
                        Use the bag data starting from up to this time [s]

  Calibration target configuration:
  --target TARGETYAML   Calibration target configuration as yaml file

  Image synchronization:
  --approx-sync MAX_DELTA_APPROXSYNC
                        Time tolerance for approximate image synchronization
                        [s] (default: 0.02)

  Calibrator settings:
  --qr-tol QRTOL        The tolerance on the factors of the QR decomposition
                        (default: 0.02)
  --mi-tol MITOL        The tolerance on the mutual information for adding an
                        image. Higher means fewer images will be added. Use -1
                        to force all images. (default: 0.2)
  --no-shuffle          Do not shuffle the dataset processing order

  Outlier filtering options:
  --no-outliers-removal
                        Disable corner outlier filtering
  --no-final-filtering  Disable filtering after all views have been processed.
  --min-views-outlier MINVIEWOUTLIER
                        Number of raw views to initialize statistics (default:
                        20)
  --use-blakezisserman  Enable the Blake-Zisserman m-estimator
  --plot-outliers       Plot the detect outliers during extraction (this could
                        be slow)

  Output options:
  --verbose             Enable (really) verbose output (disables plots)
  --show-extraction     Show the calibration target extraction. (disables
                        plots)
  --plot                Plot during calibration (this could be slow).
  --dont-show-report    Do not show the report on screen after calibration.

Output the following three files after finish calibration:

  * ``camchain-homezhangsbagstatic_10hz.yaml``
  * ``report-cam-homezhangsbagstatic_10hz.pdf``
  * ``results-cam-homezhangsbagstatic_10hz.txt``

.. tip::

  If you use camera parameters in Vins,it would be better to choose the pinhole-equi model or the omni-radtan model.If you use camera parameters in Maplab,please choose pinhole-equi model

Calibrate the pose relationship between camera and IMU coordinate system
-------------------------------------------------------------------------

* **Collect calibration data**：as calibrate the pose relationship of camera,Kalibr supports two ways to collect data,we still use ``rosbag`` here.
    * Method of collecting image: fix ``apilgrid`` calibration board, move camera
    * Make sure that the data collected is good:the brightness of the calibration board should be appropriate,too bright or too dark can't guarantee the quality of data,meanwhile do not shake too fast to avoid blurring of the image.
    * Set the imu publishing frequency to 200Hz,image to 20Hz(recommended by kalibr)
    * Fully motivate each axis of the imu,for example ,3 actions on each axis,then in the \"8-shaped\" motion

* Record camera and imu as ``dynamic.bag``.

.. code-block:: bash

  $ roslaunch mynt_eye_ros_wrapper display.launch
  $ cd bag
  $ rosbag record -O dynamic /mynteye/left/image_raw /mynteye/right/image_raw /mynteye/imu/data_raw #remember set image hz to 20hz, imu hz to 200hz

* kalibr calibration:

.. code-block:: bash

  $ kalibr_calibrate_imu_camera --cam camchain-homezhangsbagstatic_10hz.yaml --target aprilgrid.yaml --imu imu.yaml --time-calibration　--bag ~/bag/dynamic.bag

View the parameters' meanings by kalibr_calibrate_imu_camera command

.. code-block:: bash

  $ kalibr_calibrate_imu_camera --h

  Calibrate the spatial and temporal parameters of an IMU to a camera chain.

  usage:
      Example usage to calibrate a camera system against an IMU using an aprilgrid
      with temporal calibration enabled.

      kalibr_calibrate_imu_camera --bag MYROSBAG.bag --cam camchain.yaml --imu imu.yaml \
              --target aprilgrid.yaml --time-calibration

      camchain.yaml: is the camera-system calibration output of the multiple-camera
                    calibratin tool (kalibr_calibrate_cameras)

      example aprilgrid.yaml:       |  example imu.yaml: (ADIS16448)
          target_type: 'aprilgrid'  |      accelerometer_noise_density: 0.006
          tagCols: 6                |      accelerometer_random_walk: 0.0002
          tagRows: 6                |      gyroscope_noise_density: 0.0004
          tagSize: 0.088            |      gyroscope_random_walk: 4.0e-06
          tagSpacing: 0.3           |      update_rate: 200.0

  optional arguments:
    -h, --help            show this help message and exit

  Dataset source:
    --bag BAGFILE         Ros bag file containing image and imu data (rostopics
                          specified in the yamls)
    --bag-from-to bag_from_to bag_from_to
                          Use the bag data starting from up to this time [s]
    --perform-synchronization
                          Perform a clock synchronization according to 'Clock
                          synchronization algorithms for network measurements'
                          by Zhang et al. (2002).

  Camera system configuration:
    --cams CHAIN_YAML     Camera system configuration as yaml file
    --recompute-camera-chain-extrinsics
                          Recompute the camera chain extrinsics. This option is
                          exclusively recommended for debugging purposes in
                          order to identify problems with the camera chain
                          extrinsics.
    --reprojection-sigma REPROJECTION_SIGMA
                          Standard deviation of the distribution of reprojected
                          corner points [px]. (default: 1.0)

  IMU configuration:
    --imu IMU_YAMLS [IMU_YAMLS ...]
                          Yaml files holding the IMU noise parameters. The first
                          IMU will be the reference IMU.
    --imu-delay-by-correlation
                          Estimate the delay between multiple IMUs by
                          correlation. By default, no temporal calibration
                          between IMUs will be performed.
    --imu-models IMU_MODELS [IMU_MODELS ...]
                          The IMU models to estimate. Currently supported are
                          'calibrated', 'scale-misalignment' and 'scale-
                          misalignment-size-effect'.

  Calibration target:
    --target TARGET_YAML  Calibration target configuration as yaml file

  Optimization options:
    --time-calibration    Enable the temporal calibration
    --max-iter MAX_ITER   Max. iterations (default: 30)
    --recover-covariance  Recover the covariance of the design variables.
    --timeoffset-padding TIMEOFFSET_PADDING
                          Maximum range in which the timeoffset may change
                          during estimation [s] (default: 0.01)

  Output options:
    --show-extraction     Show the calibration target extraction. (disables
                          plots)
    --extraction-stepping
                          Show each image during calibration target extraction
                          (disables plots)
    --verbose             Verbose output (disables plots)
    --dont-show-report    Do not show the report on screen after calibration.

Output the follwing 4 files after finish calibration:
  * ``camchain-imucam-homezhangsbagdynamic.yaml``
  * ``imu-homezhangsbagdynamatic.yaml``
  * ``report-imucam-homezhangsbagdynamic.pdf``
  * ``results-imucam-homezhangsbagdynamic.yaml``