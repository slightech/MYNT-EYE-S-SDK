\mainpage
The VVUVCKit framework
======================

A framework for working with UVC (USB Video Class) camera controls in OS X.


Introduction
------------

Many USB Video Class Devices (UVC) have a number of built-in hardware controls.  At present, there isn't a good, high-level interface for interacting with these controls on OS X (in past years some of these controls were accessible via the QuickTime "Settings" window, but this has since been deprecated).  The VVUVCKit framework provides both a programmatic and a GUI interface for interacting with these controls, which allows you to adjust many parameters (on supported USB cameras) such as manual exposure and focus controls, in-camera brightness/contrast/gain/hue/saturation/sharpness, and manual control of white balance.  Pan/Tilt/Roll/Zoom control is also possible, but haven't been implemented yet because I don't have a camera to test with- if you do, please open an issue!

The VVUVCKit framework is essentially a heavily modified/extended version of the open source code made available with the following blog post- if you look a bit, you can still see the similar methods and workflows, so credit should go here:

http://phoboslab.org/log/2009/07/uvc-camera-control-for-mac-os-x


How to get help
---------------

Please open an "Issue" and I'll get back to you as soon as I get a chance!


What does this project include/do/make?
---------------------------------------

  * VVUVCKit is a framework that packages up the classes and resources for working with UVC controls- this framework is probably why you're here.  VVUVCKit depends on the USBBusProber (you'll need to add both to your app).
  
  * the USBBusProber framework is a re-packaging of open source code written by Apple.  I basically just took code from Apple's "Bus Prober" application and made this framework from it.  If you need a quick and easy way to access the data from "Bus Prober" in your app, you may find this useful.  You'll need to include the USBBusProber framework along with the VVUVCKit framework in your app (VVUVCKit uses USBBusProber internally).  If you're interested, the original source for the USB Prober app- as well as bunch of other neat stuff- can be found here: http://www.opensource.apple.com/source/IOUSBFamily/
  
  * UVC Test App is a simple app that demonstrates the use of the VVUVCKit framework- you pick a webcam input, and the app will display the video stream and open a window with the UVC settings that are available for the selected camera (different cameras have different capabilities).


How to use these frameworks in your Mac app
-------------------------------------------

The general idea is to compile the frameworks you want to use, add them to your XCode project so you may link against them, and then set up a build phase to copy the framework into your application bundle.  This is fairly important: most of the time when you link against a framework, the framework is expected to be installed on your OS.  VVUVCKit (and USBBusProber) are different: your application will include a compiled copy of these frameworks, so you're guaranteed that the framework won't change outside of your control (which means you won't inherit bugs or have to deal with changed APIs until you're ready to do so).  Here's the exact procedure:

  1.  In XCode, close the VVUVCKit project (if it is open), and then open your project.
  2.  In the Finder, drag the file "VVUVCKit.xcodeproj" into your project's workspace in XCode.
  3.  Switch back to XCode, and locate the "Build Phases" section for your project/application's target.
  4.  Add a dependency for "UVC Test App".  This will ensure that all the frameworks in the VVUVCKit project get compiled before your project, so there won't be any missing dependencies.
  5.  Add the "VVUVCKit" and "USBBusProber" frameworks to the "Link Binary with Libraries" section of your application (click the "+" button, then locate and add the frameworks from within your workspace).
  6.  Create a new "Copy Files" build phase, set its destination to the "Frameworks" folder, and add the frameworks you linked against in the previous step- the goal is to copy the frameworks you need into the "Frameworks" folder inside your app package.  When you click the "+" button to add the frameworks, they will be listed in the "Products" folder in the "VVUVCKit" project in your workspace.
  7.  Switch to the "Build Settings" section of your project's target, locate the "Runpath Search Paths" settings, and add the following paths: "@loader_path/../Frameworks" and "@executable_path/../Frameworks".
  8.  That's it- you're done now.  You can #import <VVUVCKit/VVUVCKit.h> now!


Documentation and Sample Code
-----------------------------

The "UVC Test App" demonstrates how to use VVUVCController to create a window with the UVC controls available for a given webcam input, and lets you view the effect your changes have on the video stream.  If you can ignore the code that does the video display, this is actually a pretty straightforward sample app.

Documentation for the VVUVCKit framework can be found here- the only class you'll probably need to work with is VVUVCController:

http://vidvox.net/rays_oddsnends/VVUVCKit_doc/

Generally speaking, you want to create the VVUVCController for a given (enabled) USB video device.  You can use the "uniqueID" property (a string containing a 16-digit hex valu) of either a QTCaptureDevice or an AVCaptureDevice.  Once you've created it, you can either open a window with a GUI for the camera's available controls (-[VVUVCController openSettingsWindow]) or get/set the parameters programmatically.  Once you're happy with the state of your camera, you can use the "createSnapshot" and "loadSnapshot:" methods to save and load the settings to/from an NSDictionary.


Licensing
---------

The source code which this project was based upon was declared as public domain in the above-linked blog post- in keeping with the spirit of free software, this repos uses the MIT license.

The USBBusProber framework is based on apple's open-source code, and as such, it retains the "apple public source license" (which is included).



