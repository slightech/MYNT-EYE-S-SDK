.. _cmake:

How to use SDK with CMake
=========================

This tutorial will create a project with CMake to start using SDK.

    You could find the project demo in ``<sdk>/samples/simple_demo/project_cmake directory`` .

Preparation
-----------

-  Windows: Install the win pack of SDK
-  Linux: build from source and ``make install``

Create Project
--------------

Add ``CMakeLists.txt`` and ``mynteye_demo.cc`` files,

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.0)

   project(mynteyed_demo VERSION 1.0.0 LANGUAGES C CXX)

   # flags

   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

   ## mynteye_demo

   add_executable(mynteye_demo mynteye_demo.cc)
   target_link_libraries(mynteye_demo mynteye ${OpenCV_LIBS})

Config Project
--------------

Add ``mynteye`` and ``OpenCV`` packages to ``CMakeLists.txt`` ,

.. code-block:: cmake

    # packages

    if(MSVC)
      set(SDK_ROOT "$ENV{MYNTEYES_SDK_ROOT}")
      if(SDK_ROOT)
        message(STATUS "MYNTEYES_SDK_ROOT: ${SDK_ROOT}")
        list(APPEND CMAKE_PREFIX_PATH
          "${SDK_ROOT}/lib/cmake"
          "${SDK_ROOT}/3rdparty/opencv/build"
        )
      else()
        message(FATAL_ERROR "MYNTEYES_SDK_ROOT not found, please install SDK firstly")
      endif()
    endif()

    ## mynteye

    find_package(mynteye REQUIRED)
    message(STATUS "Found mynteye: ${mynteye_VERSION}")

    # When SDK build with OpenCV, we can add WITH_OPENCV macro to enable some
    # features depending on OpenCV, such as ToMat().
    if(mynteye_WITH_OPENCV)
      add_definitions(-DWITH_OPENCV)
    endif()

    ## OpenCV

    # Set where to find OpenCV
    #set(OpenCV_DIR "/usr/share/OpenCV")

    # When SDK build with OpenCV, we must find the same version here.
    find_package(OpenCV REQUIRED)
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

Add ``include_directories`` and ``target_link_libraries`` to
``mynteye_demo`` target,

.. code-block:: cmake

    # targets

    include_directories(
      ${OpenCV_INCLUDE_DIRS}
    )

    ## mynteye_demo

    add_executable(mynteye_demo mynteye_demo.cc)
    target_link_libraries(mynteye_demo mynteye ${OpenCV_LIBS})

Start using SDK
---------------

Include the headers of SDK and start using its APIs, view the project demo.

Windows
~~~~~~~

Reference to “Install Build Tools” in :ref:`install_windows_exe` .

Then open ``x64 Native Tools Command Prompt for VS 2017``
command shell to build and run.

.. code-block:: bat

   mkdir _build
   cd _build

   cmake -G "Visual Studio 15 2017 Win64" ..

   msbuild.exe ALL_BUILD.vcxproj /property:Configuration=Release

   .\Release\mynteye_demo.exe

Linux
~~~~~

Open ``Terminal`` to build and run.

.. code-block:: bash

   mkdir _build
   cd _build/

   cmake ..

   make

   ./mynteye_demo
