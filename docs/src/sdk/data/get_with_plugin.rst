.. _data_get_with_plugin:

Using The Plugin To Get Data
============================

API provides a ``EnablePlugin()`` function to enable plugins under a path.

Official provided plugins for calculating binocular parallax are now available in the `MYNTEYE_BOX <http://www.myntai.com/mynteye/s/download>`_ located in the ``Plugins`` directory.

.. code-block:: none

  Plugins/
  ├─linux-x86_64/
  │  ├─libplugin_b_ocl1.2_opencv3.4.0.so
  │  ├─libplugin_g_cuda9.1_opencv2.4.13.5.so
  │  ├─libplugin_g_cuda9.1_opencv3.3.1.so
  │  └─libplugin_g_cuda9.1_opencv3.4.0.so
  ├─tegra-armv8/
  └─win-x86_64/

* The ``linux-x86_64`` directory shows the system and architecture.

  * You can find your CPU architecture from system information or ``uname -a``.

* The library name ``libplugin_*`` shows the plugin identity and the third party dependency.

  * ``b`` ``g`` is a plugin identifier, indicating that different algorithms are used.
  * ``ocl1.2`` shows it dependence on ``OpenCL 1.2``, if it exists.
  * ``cuda9.1`` shows it dependence on ``CUDA 9.1``, if it exists.
  * ``opencv3.4.0`` shows it dependence on ``OpenCV 3.4.0``, if it exists.
  * ``mynteye2.0.0`` shows it dependency on ``MYNT EYE SDK 2.0.0``, if it exists.

First, select the plugins that you are going to use depending on your situation. If you relying on third parties, please install a corresponding version.

Then, enable the plugin with the following code:

.. code-block:: c++

  auto &&api = API::Create(argc, argv);

  api->EnablePlugin("plugins/linux-x86_64/libplugin_g_cuda10.1_opencv3.4.1.so");

The path can be an absolute path or a relative path (relative to the current working directory).

Finally, just call the API to get the data as before.

.. tip::

  If the plugin is not enabled, ``api->Start(Source::VIDEO_STREAMING);`` will automatically find the appropriate plug-in in the ``<sdk>/plugins/<platform>`` directory to load.

  In other words, you can move the plug-in directory of the current platform into the ``< SDK > / plugins`` directory. To automatically load the official plugin, install the corresponding ``CUDA`` ``OpenCV`` plugin dependency, recompiling and then run ``API`` layer interface program.

Before running, please execute the following commands to ensure that  the plugin's dependency library can be searched:

.. code-block:: bash

  # Linux
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  # /usr/local/lib means the path of dependency library

  # macOS
  export DYLD_LIBRARY_PATH=/usr/local/lib:$DYLD_LIBRARY_PATH
  # /usr/local/lib means the path of dependency library

  # Windows
  set PATH=C:\opencv\x64\vc14\bin;%PATH%
  # add to PATH of system environment

In addition, the following command can be executed to check whether the dependency Library of the plug-in can be searched:

.. code-block:: bash

  # Linux
  ldd *.so
  # *.so means plugin path

  # macOS
  otool -L *.dylib
  # *.dylib means plugin path

  # Windows
  # please download Dependency Walker ，open DLL .

If the plugin's dependent library is not found, it will report an error \"Open plugin failed\" when loading.

Complete code sample, see `get_with_plugin.cc <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/samples/get_with_plugin.cc>`_ .

.. tip::

  Linux can also add a dependency library path to the system environment, so that the compiled program can run directly. (does not require ``export LD_LIBRARY_PATH`` in the terminal then run again).

  * Create a ``/etc/ld.so.conf.d/libmynteye.conf`` file and write the dependent library path.
  * Execute the ``sudo /sbin/ldconfig`` command in the terminal and refresh the cache.

  .. literalinclude:: ../../files/libmynteye.conf
    :caption: e.g. libmynteye.conf
