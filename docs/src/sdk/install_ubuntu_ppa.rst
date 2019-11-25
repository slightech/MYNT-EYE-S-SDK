.. _sdk_install_ubuntu_ppa:

Ubuntu PPA Installation
============================

.. only:: html

  =============== =============== ===============
  Ubuntu 14.04    Ubuntu 16.04    Ubuntu 18.04
  =============== =============== ===============
  |build_passing| |build_passing| |build_passing|
  =============== =============== ===============

  .. |build_passing| image:: https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat

.. only:: latex

  =============== =============== ===============
  Ubuntu 14.04    Ubuntu 16.04    Ubuntu 18.04
  =============== =============== ===============
  ✓               ✓               ✓
  =============== =============== ===============

x64 PPA installation
--------------------

.. code-block:: bash

  $ sudo add-apt-repository ppa:slightech/mynt-eye-s-sdk
  $ sudo apt-get update
  $ sudo apt-get install mynt-eye-s-sdk

armv8 PPA installation
-----------------------

.. code-block:: bash

  $ sudo add-apt-repository ppa:slightech/mynt-eye-s-sdk-arm
  $ sudo apt-get update
  $ sudo apt-get install mynt-eye-s-sdk

Run samples
------------

.. tip::

  samples path: /opt/mynt-eye-s-sdk/samples

.. code-block:: bash

  $ cd /opt/mynt-eye-s-sdk/samples
  $ ./camera_with_junior_device_api