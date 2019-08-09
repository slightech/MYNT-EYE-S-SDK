.. _okvis:

How To Use In `OKVIS <https://github.com/ethz-asl/okvis>`_
=============================================================

If you wanna run OKVIS with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install it.
2. Install dependencies and build MYNT-EYE-OKVIS-Sample follow the procedure of the original OKVIS.
3. Update camera parameters to ``<OKVIS>/config/config_mynteye.yaml``.
4. Run OKVIS using MYNT® EYE.

.. tip::

  OKVIS doesn't support ARM right now


Install MYNTEYE OKVIS
---------------------

First install dependencies based on the original OKVIS, and the follow:

.. code-block:: bash

  sudo apt-get install libgoogle-glog-dev

  git clone -b mynteye https://github.com/slightech/MYNT-EYE-OKVIS-Sample.git
  cd MYNT-EYE-OKVIS-Sample/
  mkdir build && cd build
  cmake ..
  make

Get camera calibration parameters
----------------------------------

Through the ``GetIntrinsics()`` and ``GetExtrinsics()`` function of the `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ API, you can get the camera calibration parameters of the currently open device, follow the steps:

.. code-block:: bat

  cd MYNT-EYE-S-SDK
  ./samples/_output/bin/tutorials/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained , and then update to `here <https://github.com/slightech/MYNT-EYE-OKVIS-Sample/blob/mynteye/config/config_mynteye_s.yaml>`_ .

.. tip::

  You can get the camera model of device when get camera calibration parameters, if model is equidistant you need calibrate pinhole model by yourself or reference :ref:`write_img_params` to write a default pinhole config file to your device.

.. code-block:: bash

  distortion_coefficients: [coeffs]   # only first four parameters of coeffs need to be filled
  focal_length: [fx, fy]
  principal_point: [cx, cy]
  distortion_type: radialtangential

Run MYNTEYE OKVIS
---------------------

Go to ``MYNT-EYE-OKVIS-Sample/build`` folder and Run the application ``okvis_app_mynteye_s`` :

.. code-block:: bash

  cd MYNT-EYE-OKVIS-Sample/build
  ./okvis_app_mynteye_s ../config/config_mynteye_s.yaml
