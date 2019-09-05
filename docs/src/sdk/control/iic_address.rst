.. _iic_address:

Set IIC Address
================

Using the ``SetOptionValue()`` function in the API, you can set various control values for the current device.

To set the IIC address, set ``Option::IIC_ADDRESS_SETTING``.


.. Attention::
  Only support S21XX

Reference Code:

s2XX：

.. code-block:: c++

  auto &&api = API::Create(argc, argv);
  if (!api) return 1;
  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;
  api->ConfigStreamRequest(request);
  Model model = api->GetModel();
  if (model == Model::STANDARD210A || model == Model::STANDARD2) {
    api->SetOptionValue(Option::IIC_ADDRESS_SETTING, 0x31);
    LOG(INFO) << "Set iic address to " << std::hex << "0x"
              << api->GetOptionValue(Option::IIC_ADDRESS_SETTING);
  }


Reference running results on Linux:

s21XX：

.. code-block:: bash

  $ ./samples/_output/bin/ctrl_iic_adress
  I/utils.cc:30 Detecting MYNT EYE devices
  I/utils.cc:40 MYNT EYE devices:
  I/utils.cc:43   index: 0, name: MYNT-EYE-S210A, sn: 07C41A190009071F
  I/utils.cc:51 Only one MYNT EYE device, select index: 0
  I/utils.cc:79 MYNT EYE devices:
  I/utils.cc:82   index: 0, request: width: 1280, height: 400, format: Format::BGR888, fps: 10
  I/utils.cc:82   index: 1, request: width: 1280, height: 400, format: Format::BGR888, fps: 20
  I/utils.cc:82   index: 2, request: width: 1280, height: 400, format: Format::BGR888, fps: 30
  I/utils.cc:82   index: 3, request: width: 1280, height: 400, format: Format::BGR888, fps: 60
  I/utils.cc:82   index: 4, request: width: 2560, height: 800, format: Format::BGR888, fps: 10
  I/utils.cc:82   index: 5, request: width: 2560, height: 800, format: Format::BGR888, fps: 20
  I/utils.cc:82   index: 6, request: width: 2560, height: 800, format: Format::BGR888, fps: 30
  I/utils.cc:93 There are 7 stream requests, select index:
  3
  I/imu_range.cc:51 Set iic address to 0x31

After the sample program finishes running with ``ESC/Q``.
Complete code samples，please see `ctrl_iic_address.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/ctrl_iic_address.cc>`_ .
