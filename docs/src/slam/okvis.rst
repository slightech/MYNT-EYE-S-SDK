.. _slam_okvis:

How to use in `OKVIS <https://github.com/ethz-asl/okvis>`_
=============================================================

If you wanna run OKVIS with MYNT EYE camera, please follow the steps:
----------------------------------------------------------------------

1. Download `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ and install it.
2. Install dependencies and build MYNT-EYE-OKVIS-Sample follow the procedure of the original OKVIS.
3. Update camera parameters to ``<OKVIS>/config/config_mynteye.yaml``.
4. Run OKVIS using MYNT® EYE.

Install MYNTEYE OKVIS
---------------------

First install dependencies based on the original OKVIS, and the follow:

.. code-block:: bash

  git clone -b mynteye-s https://github.com/slightech/MYNT-EYE-OKVIS-Sample.git
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j4

Get camera calibration parameters
----------------------------------

Through the ``GetIntrinsics()`` and ``GetExtrinsics()`` function of the `MYNT-EYE-S-SDK <https://github.com/slightech/MYNT-EYE-S-SDK.git>`_ API, you can get the camera calibration parameters of the currently open device, follow the steps:

.. code-block:: bat

  cd MYNT-EYE-S-SDK
  ./samples/_output/bin/tutorials/get_img_params

After running the above type, pinhole's ``distortion_parameters`` and ``projection_parameters`` is obtained, then update to `here <https://github.com/slightech/MYNT-EYE-OKVIS-Sample/blob/mynteye-s/config/config_mynteye.yaml>`_ .

Run MYNTEYE OKVIS
---------------------

Go to ``MYNT-EYE-OKVIS-Sample/build`` folder and Run the application ``okvis_app_mynteye_s`` :

.. code-block:: bash

  cd MYNT-EYE-OKVIS-Sample/build
  ./okvis_app_mynteye_s ../config/config_mynteye.yaml
