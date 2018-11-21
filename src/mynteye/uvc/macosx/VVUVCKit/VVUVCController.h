#import <Foundation/Foundation.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#import <USBBusProber/USBBusProber.h>
//#import <QTKit/QTKit.h>
#import "VVUVCKitStringAdditions.h"

/**
\defgroup VVUVCController
*/


/*
@protocol VVUVCControllerDelegate
	- (void) VVUVCControllerParamsUpdated:(id)c;
@end
*/


/**
\ingroup VVUVCController
Auto-exposure modes described by the USB spec, put in a typedef/enum for convenience
*/
typedef enum	{
	UVC_AEMode_Undefined = 0x00,	///	undefined auto exposure mode
	UVC_AEMode_Manual = 0x01,	///	manual exposure, manual iris
	UVC_AEMode_Auto = 0x02,	///	auto exposure, auto iris
	UVC_AEMode_ShutterPriority = 0x04,	///	manual exposure, auto iris
	UVC_AEMode_AperturePriority = 0x08	///	auto exposure, manual iris
} UVC_AEMode;


/*		this struct contains all the info necessary to get/set vals from a video control parameter (either terminal/hardware or 
		processing/software)- but it does not contain info about the value at all!  think of this struct as a sort of function 
		description for the hardware which will be sent out directly via USB.  instances of this struct are populated by values 
		from the USB specification!			*/
typedef struct {
	int		unit;	//	describes whether terminal/hardware or processing/software
	int		selector;	//	the address of the "parameter" being changed- 
	int		intendedSize;
	BOOL	hasMin;	//	whether or not the video control parameter described by this struct has a min val
	BOOL	hasMax;	//	whether or not the video control parameter described by this struct has a max
	BOOL	hasDef;	//	whether or not the video control parameter described by this struct has a default
	BOOL	isSigned;	//	whether or not the video control parameter described by this struct is a signed val
	BOOL	isRelative;	//	whether or not the video control parameter described by this struct is a relative val
} uvc_control_info_t;


/*	these variables contain enough info to send data to/get data from the implied attribute.  the 
	variables are global to the class (the contents won't change from instance to instance), and 
	conceptually act like function descriptions (instances of this class can pass references to these 
	variables, which can be used to get/set values).  these are populated when the class is 
	initialized by values described in the USB specification.  if uvc_control_info_t is a function 
	description, these variables are essentially pointers to a bunch of different functions.		*/
extern uvc_control_info_t	_scanCtrl;
extern uvc_control_info_t	_autoExposureModeCtrl;
extern uvc_control_info_t	_autoExposurePriorityCtrl;
extern uvc_control_info_t	_exposureTimeCtrl;
extern uvc_control_info_t	_irisCtrl;
extern uvc_control_info_t	_autoFocusCtrl;
extern uvc_control_info_t	_focusCtrl;
extern uvc_control_info_t	_zoomCtrl;
extern uvc_control_info_t	_panTiltCtrl;
extern uvc_control_info_t	_panTiltRelCtrl;
extern uvc_control_info_t	_rollCtrl;
extern uvc_control_info_t	_rollRelCtrl;

extern uvc_control_info_t	_backlightCtrl;
extern uvc_control_info_t	_brightCtrl;
extern uvc_control_info_t	_contrastCtrl;
extern uvc_control_info_t	_gainCtrl;
extern uvc_control_info_t	_powerLineCtrl;
extern uvc_control_info_t	_autoHueCtrl;
extern uvc_control_info_t	_hueCtrl;
extern uvc_control_info_t	_saturationCtrl;
extern uvc_control_info_t	_sharpnessCtrl;
extern uvc_control_info_t	_gammaCtrl;
extern uvc_control_info_t	_whiteBalanceAutoTempCtrl;
extern uvc_control_info_t	_whiteBalanceTempCtrl;


/*		this struct describes a parameter- it contains a pointer to the control info that describes 
which parameter, as well as the min/max/default/current value. an instance of this struct will 
contain the value of the parameter and the uvc_control_info_t struct necessary to communicate with 
this parameter in a camera.		*/
typedef struct	{
	BOOL	supported;	//	if YES, this parameter is supported. if NO, either the camera doesn't support this parameter, or the "inputTerminalID" or "processingUnitID" of the camera is wrong!
	long	min;	//	the paramter's actual min val
	long	max;	//	the parameter's actual max val
	long	val;	//	the parameter's actual val
	long	def;	//	the parameter's default val
	int		actualSize;
	uvc_control_info_t	*ctrlInfo;
} uvc_param;





///	An instance of VVUVCController will control the UVC params for a single USB video device.  This is probably the only class you'll have to create or work with in this framework.
/**
\ingroup VVUVCController
This is probably the only class you'll have to work with in this framework.  The basic idea is that you create a VVUVCController for an enabled USB video device, and then either tell the controller to open its settings window or interact with it programmatically.  If you're looking for a more "embedded" feel, you can remove the VVUVCController's "settingsView" from its superview and add it into your application's NSView hierarchy.
*/
@interface VVUVCController : NSObject {
	IOUSBInterfaceInterface190		**interface;
	UInt32							deviceLocationID;
	UInt8							interfaceNumber;	//	pulled from interface on generalInit!
	int					inputTerminalID;	//	the "address" of the terminal unit, which handles hardware controls like aperture/focus.  if this val is wrong, the hardware controls won't be available.
	int					processingUnitID;	//	the "address" of the processing unit, which handles software controls like contrast/hue.  if this val is wrong, the software controls won't be available.
	
	//id					<VVUVCControllerDelegate>delegate;
	uvc_param			scanningMode;
	uvc_param			autoExposureMode;	//	mode functionality described by the type UVC_AEMode
	uvc_param			autoExposurePriority;	//	if 1, framerate may be varied.  if 0, framerate must remain constant.
	uvc_param			exposureTime;
	uvc_param			iris;
	uvc_param			autoFocus;
	uvc_param			focus;
	uvc_param			zoom;
	uvc_param			panTilt;
	uvc_param			panTiltRel;
	uvc_param			roll;
	uvc_param			rollRel;
	
	uvc_param			backlight;
	uvc_param			bright;
	uvc_param			contrast;
	uvc_param			gain;
	uvc_param			powerLine;
	uvc_param			autoHue;
	uvc_param			hue;
	uvc_param			saturation;
	uvc_param			sharpness;
	uvc_param			gamma;
	uvc_param			autoWhiteBalance;
	uvc_param			whiteBalance;
	
	//	this class has its own .nib which contains a UI for interacting with the class that may be opened directly in a window, or accessed as an NSView instance for use in other UIs/software
	NSNib				*theNib;
	NSArray				*nibTopLevelObjects;
	
	IBOutlet id			uiCtrlr;	//	created & owned by the nib!
	IBOutlet NSWindow	*settingsWindow;	//	by default, the UI is in a window (it's easiest to just open and close it)
	IBOutlet NSView		*settingsView;	//	you can also access the view which contains the UI so you can embed it in other apps
}

///	Use this method to init an instance of VVUVCController from an NSString returned by the AVFoundation or QTCapture APIs as the device's unique ID.
/**
@param n The "deviceIDString" is a string returned by QuickTime and AVFoundation as the unique ID for the USB video device.  Technically, this is a hex value with sixteen digits (16 hex digits = an unsigned 64-bit integer).  The first 8 hex digits is the USB device's "locationID", the next 4 hex digits is the device's vendor ID, and the last 4 digits are the device's product ID.  Only the locationID is needed to create the necessary USB interfaces...
*/
- (id) initWithDeviceIDString:(NSString *)n;
///	Use this method to init an instance of VVUVCController from the USB location ID.
/**
@param locationID The location ID of the USB device you want this instance of VVUVCController to control.
*/
- (id) initWithLocationID:(UInt32)locationID;
- (IOUSBInterfaceInterface190 **) _getControlInferaceWithDeviceInterface:(IOUSBDeviceInterface **)deviceInterface;
- (void) generalInit;

///	Returns a mutable dict representing the current state of the video input parameters
- (NSMutableDictionary *) createSnapshot;
///	Loads a saved state dict created with the "createSnapshot" method
- (void) loadSnapshot:(NSDictionary *)s;

- (BOOL) _sendControlRequest:(IOUSBDevRequest *)controlRequest;
- (int) _requestValType:(int)requestType forControl:(const uvc_control_info_t *)ctrl returnVal:(void **)ret;
- (BOOL) _setBytes:(void *)bytes sized:(int)size toControl:(const uvc_control_info_t *)ctrl;
- (void) _populateAllParams;	//	populates all the uvc_param variables in this instance, loading their min/max/default vals and determining if they're supported or not
- (void) _populateParam:(uvc_param *)param;
- (BOOL) _pushParamToDevice:(uvc_param *)param;
- (void) _resetParamToDefault:(uvc_param *)param;

///	Resets the parameters to their default values.  The default values are supplied by/stored in the device.
- (void) resetParamsToDefaults;
///	Opens a window with a GUI for interacting with the camera parameters
- (void) openSettingsWindow;
///	Closes the GUI window (if it's open).
- (void) closeSettingsWindow;

- (void) setInterlaced:(BOOL)n;
- (BOOL) interlaced;
- (BOOL) interlacedSupported;
- (void) resetInterlaced;
///	Sets the auto exposure mode using one of the basic auto exposure modes defined in the header (vals pulled from the USB spec)
- (void) setAutoExposureMode:(UVC_AEMode)n;
///	Gets the auto exposure mode
- (UVC_AEMode) autoExposureMode;
///	Whether or not this camera supports the use of alternate auto exposure modes
- (BOOL) autoExposureModeSupported;
///	Resets the auto exposure mode to the hardware-defined default
- (void) resetAutoExposureMode;
///	Sets whether or not auto exposure will be given priority
- (void) setAutoExposurePriority:(BOOL)n;
///	Gets whether or not the camera is giving auto exposure priority
- (BOOL) autoExposurePriority;
///	Whether or not this camera supports the use of auto exposure priority
- (BOOL) autoExposurePrioritySupported;
///	Resets the auto exposure priority to the hardware-defined default
- (void) resetAutoExposurePriority;

///	Sets the exposure time to the passed value
- (void) setExposureTime:(long)n;
///	Gets the current exposure time value being used by the camera
- (long) exposureTime;
///	Whether or not this camera supports the exposure time parameter
- (BOOL) exposureTimeSupported;
///	Resets the exposure time value to the hardware-defined default
- (void) resetExposureTime;
///	The min exposure time value
- (long) minExposureTime;
///	The max exposure time value
- (long) maxExposureTime;
///	Sets the iris to the passed value
- (void) setIris:(long)n;
///	Gets the current iris value being used by the camera
- (long) iris;
///	Whether or not this camera supports the iris parameter
- (BOOL) irisSupported;
///	Resets the iris value to the hardware-defined default
- (void) resetIris;
///	The min iris value
- (long) minIris;
///	The max iris value
- (long) maxIris;
///	Sets the auto focus to the passed value
- (void) setAutoFocus:(BOOL)n;
///	Gets the auto focus value being used by the camera
- (BOOL) autoFocus;
///	Whether or not this camera supports the auto focus parameter
- (BOOL) autoFocusSupported;
///	Resets the auto focus value to the hardware-defined default.
- (void) resetAutoFocus;
///	Sets the focus value
- (void) setFocus:(long)n;
///	Gets the focus value currently being used by the camera
- (long) focus;
///	Whether or not this camera supports the focus parameter
- (BOOL) focusSupported;
///	Resets the focus value to the hardware-defined default
- (void) resetFocus;
///	The min focus value
- (long) minFocus;
///	The max focus value
- (long) maxFocus;
///	Sets the zoom value
- (void) setZoom:(long)n;
///	Gets the current zoom value being used by the camera
- (long) zoom;
///	Whether or not this camera supports the zoom parameter
- (BOOL) zoomSupported;
///	Resets the zoom value to the hardware-defined default
- (void) resetZoom;
///	The min zoom value
- (long) minZoom;
///	The max zoom value
- (long) maxZoom;

//	pan/tilt/roll aren't enabled
- (BOOL) panSupported;
- (BOOL) tiltSupported;
- (BOOL) rollSupported;

///	Sets the backlight to the passed value
- (void) setBacklight:(long)n;
///	Gets the backlight value currently being used by the camera
- (long) backlight;
///	Whether or not this camera supports the backlight parameter
- (BOOL) backlightSupported;
///	Resets the backlight value to the hardware-defined default
- (void) resetBacklight;
///	The min backlight value
- (long) minBacklight;
///	The max backlight value
- (long) maxBacklight;
///	Sets the bright value to the passed value
- (void) setBright:(long)n;
///	Gets the bright value currently being used by the camera
- (long) bright;
///	Whether or not this camera supports the bright parameter
- (BOOL) brightSupported;
///	Resets the bright parameter to the hardware-defined default
- (void) resetBright;
///	The min bright value
- (long) minBright;
///	The max bright value
- (long) maxBright;
///	Sets the contrast to the passed value
- (void) setContrast:(long)n;
///	Gets the contrast value currently being used by the camera
- (long) contrast;
///	Whether or not this camera supports the contrast parameter
- (BOOL) contrastSupported;
///	Resets the contrast to the hardware-defined default
- (void) resetContrast;
///	The min contrast value
- (long) minContrast;
///	The max contrast value
- (long) maxContrast;
///	Sets the gain to the passed value
- (void) setGain:(long)n;
///	Gets the gain value currently being used by the camera
- (long) gain;
///	Whether or not this camera supports the gain parameter
- (BOOL) gainSupported;
///	Resets the gain value to the hardware-defined default
- (void) resetGain;
///	The min gain value
- (long) minGain;
///	The max gain value
- (long) maxGain;
///	Sets the powerline to the passed value
- (void) setPowerLine:(long)n;
///	Gets the powerline value currently being used by the camera
- (long) powerLine;
///	Whether or not this camera supports the powerline parameter
- (BOOL) powerLineSupported;
///	Resets the powerline value to the hardware-defined default
- (void) resetPowerLine;
///	The min powerline value
- (long) minPowerLine;
///	The max powerline value
- (long) maxPowerLine;
///	Sets the auto hue to the passed value
- (void) setAutoHue:(BOOL)n;
///	The auto hue value currently being used by the camera
- (BOOL) autoHue;
///	Whether or not this camera supports the auto hue parameter
- (BOOL) autoHueSupported;
///	Resets the auto hue parameter to the hardware-defined default
- (void) resetAutoHue;
///	Sets the hue to the passed value
- (void) setHue:(long)n;
///	Gets the hue value currently being used by the camera
- (long) hue;
///	Whether or not this camera supports the hue parameter
- (BOOL) hueSupported;
///	Resets the hue parameter to the hardware-defined default
- (void) resetHue;
///	The min hue value
- (long) minHue;
///	The max hue value
- (long) maxHue;
///	Sets the saturation to the passed value
- (void) setSaturation:(long)n;
///	Gets the saturation value currently being used by the camera
- (long) saturation;
///	Whether or not this camera supports the saturation parameter
- (BOOL) saturationSupported;
///	Resets the saturation to the hardware-defined default
- (void) resetSaturation;
///	The min saturation value
- (long) minSaturation;
///	The max saturation value
- (long) maxSaturation;
///	Sets the sharpness to the passed value
- (void) setSharpness:(long)n;
///	Gets the sharpness value currently being used by the camera
- (long) sharpness;
///	Whether or not this camera supports the sharpness parameter
- (BOOL) sharpnessSupported;
///	Resets the sharpness to the hardware-defined default
- (void) resetSharpness;
///	The min sharpness value
- (long) minSharpness;
///	The max sharpness value
- (long) maxSharpness;
///	Sets the gamma to the passed value
- (void) setGamma:(long)n;
///	Gets the gamma value currently being used by the camera
- (long) gamma;
///	Whether or not this camera supports the gamma parameter
- (BOOL) gammaSupported;
///	Resets the gamma value to the hardware-defined default
- (void) resetGamma;
///	The min gamma value
- (long) minGamma;
///	The max gamma value
- (long) maxGamma;
///	Sets the auto white balance to the passed value
- (void) setAutoWhiteBalance:(BOOL)n;
///	Gets the auto white balance value currently being used by the camera
- (BOOL) autoWhiteBalance;
///	Whether or not this camera supports the auto white balance parameter
- (BOOL) autoWhiteBalanceSupported;
///	Resets the auto white balance to the hardware-defined default
- (void) resetAutoWhiteBalance;
///	Sets the white balance to the passed value
- (void) setWhiteBalance:(long)n;
///	Gets the white balance value currently being used by the camera
- (long) whiteBalance;
///	Whether or not this camera supports the white balance parameter
- (BOOL) whiteBalanceSupported;
///	Resets the white balance value to the hardware-defined default
- (void) resetWhiteBalance;
///	The min white balance value
- (long) minWhiteBalance;
///	The max white balance value
- (long) maxWhiteBalance;


@end
