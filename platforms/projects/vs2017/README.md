# How to use MYNT® EYE S SDK with Visual Studio 2017

This tutorial will create a project with Visual Studio 2017 to start using MYNT® EYE S SDK.

## Preparation

Install the win pack of MYNT® EYE S SDK.

## Create Project

Open Visual Studio 2017, then `File > New > Project`,

![](images/1_new_pro.png)

Select "Windows Console Application", set the project's name and location, click "OK",

![](images/2_new_pro.png)

Finally, you will see the new project like this,

![](images/3_new_pro.png)

## Config Properties

Right click the project, and open its "Properties" window,

![](images/4_config.png)

Change "Configuration" to "All Configurations", then add the following paths to "Additional Include Directories",

```bash
$(MYNTEYES_SDK_ROOT)\include
$(MYNTEYES_SDK_ROOT)\3rdparty\opencv\build\include
```

![](images/5_config_include.png)

Add the following paths to "Additional Library Directories",

```bash
$(MYNTEYES_SDK_ROOT)\lib
$(MYNTEYES_SDK_ROOT)\3rdparty\opencv\build\x64\vc15\lib
```

![](images/6_config_lib_dir.png)


Add the following libs to "Additional Dependencies",

```bash
mynteye.lib
opencv_world343.lib
```

![](images/7_config_lib.png)

If you wanna debug, could change "Configuration" to "Debug" and add these debug libs,

```bash
mynteyed.lib
opencv_world343d.lib
```

![](images/8_config_debug_lib.png)

## Start using SDK

Include the headers of SDK and start using its APIs,

![](images/9_run_x64.png)

Select "Release x64" or "Debug x64" to run the project.

<!--
![](images/10_path.png)
-->
