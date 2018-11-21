
#import "VVUVCController.h"
#import "VVUVCUIController.h"




//#define NSLogParam(n,p) NSLog(@"%@, (%ld)-[%ld]-(%ld), %ld",n,p.min,p.val,p.max,p.def)
#define NSLogParam(n,p)	{														\
	if (p.supported)	{														\
		if (p.ctrlInfo->isRelative)											\
			NSLog(@"%@, supported but relative!",n);							\
		else																	\
			NSLog(@"%@, (%ld)-[%ld]-(%ld), %ld",n,p.min,p.val,p.max,p.def);		\
	}																			\
	else	{																	\
		NSLog(@"%@ is unsupported",n);											\
	}																			\
}
/*		these values are used to signify whether the uvc_control_info struct affects a hardware parameter (like focus), 
which is one block of "function calls", or whether it affects a software parameter (like brightness), which is in another 
block of "function calls".  one block is the "input terminal", the other block is the "processing unit".  the actual 
addresses of these "blocks" is different from camera to camera ("inputTerminalID" and "processingUnitID" vars in 
VVUVCControl class contain the actual per-camera addresses of these blocks)- these defines just indicate which var to use!		*/
#define UVC_INPUT_TERMINAL_ID 0x01
#define UVC_PROCESSING_UNIT_ID 0x02		//	other cams i've used so far




/*	IMPORTANT: ALL THESE DEFINES WERE TAKEN FROM THE USB SPECIFICATION:
	http://www.usb.org/developers/docs/devclass_docs/USB_Video_Class_1_1_090711.zip			*/




#define UVC_CONTROL_INTERFACE_CLASS 0x0E
#define UVC_CONTROL_INTERFACE_SUBCLASS 0x01

//	video class-specific request codes
#define UVC_SET_CUR	0x01
#define UVC_GET_CUR	0x81
#define UVC_GET_MIN	0x82
#define UVC_GET_MAX	0x83
#define UVC_GET_RES 0x84
#define UVC_GET_LEN 0x85
#define UVC_GET_INFO 0x86
#define UVC_GET_DEF 0x87

//	camera terminal control selectors
typedef enum	{
	UVC_CT_CONTROL_UNDEFINED = 0x00,
	UVC_CT_SCANNING_MODE_CONTROL = 0x01,
	UVC_CT_AE_MODE_CONTROL = 0x02,
	UVC_CT_AE_PRIORITY_CONTROL = 0x03,
	UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL = 0x04,
	UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL = 0x05,
	UVC_CT_FOCUS_ABSOLUTE_CONTROL = 0x06,
	UVC_CT_FOCUS_RELATIVE_CONTROL = 0x07,
	UVC_CT_FOCUS_AUTO_CONTROL = 0x08,
	UVC_CT_IRIS_ABSOLUTE_CONTROL = 0x09,
	UVC_CT_IRIS_RELATIVE_CONTROL = 0x0A,
	UVC_CT_ZOOM_ABSOLUTE_CONTROL = 0x0B,
	UVC_CT_ZOOM_RELATIVE_CONTROL = 0x0C,
	UVC_CT_PANTILT_ABSOLUTE_CONTROL = 0x0D,
	UVC_CT_PANTILT_RELATIVE_CONTROL = 0x0E,
	UVC_CT_ROLL_ABSOLUTE_CONTROL = 0x0F,
	UVC_CT_ROLL_RELATIVE_CONTROL = 0x10
} UVC_CT_t;

//	UVC processing unit control selectors
typedef enum	{
	UVC_PU_CONTROL_UNDEFINED = 0x00,
	UVC_PU_BACKLIGHT_COMPENSATION_CONTROL = 0x01,
	UVC_PU_BRIGHTNESS_CONTROL = 0x02,
	UVC_PU_CONTRAST_CONTROL = 0x03,
	UVC_PU_GAIN_CONTROL = 0x04,
	UVC_PU_POWER_LINE_FREQUENCY_CONTROL = 0x05,
	UVC_PU_HUE_CONTROL = 0x06,
	UVC_PU_SATURATION_CONTROL = 0x07,
	UVC_PU_SHARPNESS_CONTROL = 0x08,
	UVC_PU_GAMMA_CONTROL = 0x09,
	UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL = 0x0A,
	UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL = 0x0B,
	UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL = 0x0C,
	UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL = 0x0D,
	UVC_PU_DIGITAL_MULTIPLIER_CONTROL = 0x0E,
	UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL = 0x0F,
	UVC_PU_HUE_AUTO_CONTROL = 0x10,
	UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL = 0x11,
	UVC_PU_ANALOG_LOCK_STATUS_CONTROL = 0x12
} UVC_PU_t;


uvc_control_info_t	_scanCtrl;
uvc_control_info_t	_autoExposureModeCtrl;
uvc_control_info_t	_autoExposurePriorityCtrl;
uvc_control_info_t	_exposureTimeCtrl;
uvc_control_info_t	_irisCtrl;
uvc_control_info_t	_autoFocusCtrl;
uvc_control_info_t	_focusCtrl;
uvc_control_info_t	_zoomCtrl;
uvc_control_info_t	_panTiltCtrl;
uvc_control_info_t	_panTiltRelCtrl;
uvc_control_info_t	_rollCtrl;
uvc_control_info_t	_rollRelCtrl;

uvc_control_info_t	_backlightCtrl;
uvc_control_info_t	_brightCtrl;
uvc_control_info_t	_contrastCtrl;
uvc_control_info_t	_gainCtrl;
uvc_control_info_t	_powerLineCtrl;
uvc_control_info_t	_autoHueCtrl;
uvc_control_info_t	_hueCtrl;
uvc_control_info_t	_saturationCtrl;
uvc_control_info_t	_sharpnessCtrl;
uvc_control_info_t	_gammaCtrl;
uvc_control_info_t	_whiteBalanceAutoTempCtrl;
uvc_control_info_t	_whiteBalanceTempCtrl;




@implementation VVUVCController


+ (void) load	{
	_scanCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_scanCtrl.selector = UVC_CT_SCANNING_MODE_CONTROL;
	_scanCtrl.intendedSize = 1;
	_scanCtrl.hasMin = NO;
	_scanCtrl.hasMax = NO;
	_scanCtrl.hasDef = NO;
	_scanCtrl.isSigned = NO;
	_scanCtrl.isRelative = NO;
	_autoExposureModeCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_autoExposureModeCtrl.selector = UVC_CT_AE_MODE_CONTROL;
	_autoExposureModeCtrl.intendedSize = 1;
	_autoExposureModeCtrl.hasMin = NO;
	_autoExposureModeCtrl.hasMax = NO;
	_autoExposureModeCtrl.hasDef = YES;
	_autoExposureModeCtrl.isSigned = NO;
	_autoExposureModeCtrl.isRelative = NO;
	_autoExposurePriorityCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_autoExposurePriorityCtrl.selector = UVC_CT_AE_PRIORITY_CONTROL;
	_autoExposurePriorityCtrl.intendedSize = 1;
	_autoExposurePriorityCtrl.hasMin = NO;
	_autoExposurePriorityCtrl.hasMax = NO;
	_autoExposurePriorityCtrl.hasDef = NO;
	_autoExposurePriorityCtrl.isSigned = NO;
	_autoExposurePriorityCtrl.isRelative = NO;
	_exposureTimeCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_exposureTimeCtrl.selector = UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL;
	_exposureTimeCtrl.intendedSize = 4;
	_exposureTimeCtrl.hasMin = YES;
	_exposureTimeCtrl.hasMax = YES;
	_exposureTimeCtrl.hasDef = YES;
	_exposureTimeCtrl.isSigned = NO;
	_exposureTimeCtrl.isRelative = NO;
	_irisCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_irisCtrl.selector = UVC_CT_IRIS_ABSOLUTE_CONTROL;
	_irisCtrl.intendedSize = 2;
	_irisCtrl.hasMin = YES;
	_irisCtrl.hasMax = YES;
	_irisCtrl.hasDef = YES;
	_irisCtrl.isSigned = NO;
	_irisCtrl.isRelative = NO;
	_autoFocusCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_autoFocusCtrl.selector = UVC_CT_FOCUS_AUTO_CONTROL;
	_autoFocusCtrl.intendedSize = 1;
	_autoFocusCtrl.hasMin = NO;
	_autoFocusCtrl.hasMax = NO;
	_autoFocusCtrl.hasDef = YES;
	_autoFocusCtrl.isSigned = NO;
	_autoFocusCtrl.isRelative = NO;
	_focusCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_focusCtrl.selector = UVC_CT_FOCUS_ABSOLUTE_CONTROL;
	_focusCtrl.intendedSize = 2;
	_focusCtrl.hasMin = YES;
	_focusCtrl.hasMax = YES;
	_focusCtrl.hasDef = YES;
	_focusCtrl.isSigned = NO;
	_focusCtrl.isRelative = NO;
	_zoomCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_zoomCtrl.selector = UVC_CT_ZOOM_ABSOLUTE_CONTROL;
	_zoomCtrl.intendedSize = 2;
	_zoomCtrl.hasMin = YES;
	_zoomCtrl.hasMax = YES;
	_zoomCtrl.hasDef = YES;
	_zoomCtrl.isSigned = NO;
	_zoomCtrl.isRelative = NO;
	_panTiltCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_panTiltCtrl.selector = UVC_CT_PANTILT_ABSOLUTE_CONTROL;
	_panTiltCtrl.intendedSize = 8;
	_panTiltCtrl.hasMin = YES;
	_panTiltCtrl.hasMax = YES;
	_panTiltCtrl.hasDef = YES;
	_panTiltCtrl.isSigned = YES;
	_panTiltCtrl.isRelative = NO;
	_panTiltRelCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_panTiltRelCtrl.selector = UVC_CT_PANTILT_RELATIVE_CONTROL;
	_panTiltRelCtrl.intendedSize = 1;
	_panTiltRelCtrl.hasMin = YES;
	_panTiltRelCtrl.hasMax = YES;
	_panTiltRelCtrl.hasDef = YES;
	_panTiltRelCtrl.isSigned = YES;
	_panTiltRelCtrl.isRelative = YES;
	_rollCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_rollCtrl.selector = UVC_CT_ROLL_ABSOLUTE_CONTROL;
	_rollCtrl.intendedSize = 2;
	_rollCtrl.hasMin = YES;
	_rollCtrl.hasMax = YES;
	_rollCtrl.hasDef = YES;
	_rollCtrl.isSigned = YES;
	_rollCtrl.isRelative = NO;
	_rollRelCtrl.unit = UVC_INPUT_TERMINAL_ID;
	_rollRelCtrl.selector = UVC_CT_ROLL_RELATIVE_CONTROL;
	_rollRelCtrl.intendedSize = 1;
	_rollRelCtrl.hasMin = YES;
	_rollRelCtrl.hasMax = YES;
	_rollRelCtrl.hasDef = YES;
	_rollRelCtrl.isSigned = YES;
	_rollRelCtrl.isRelative = YES;
	
	_backlightCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_backlightCtrl.selector = UVC_PU_BACKLIGHT_COMPENSATION_CONTROL;
	_backlightCtrl.intendedSize = 2;
	_backlightCtrl.hasMin = YES;
	_backlightCtrl.hasMax = YES;
	_backlightCtrl.hasDef = YES;
	_backlightCtrl.isSigned = NO;
	_backlightCtrl.isRelative = NO;
	_brightCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_brightCtrl.selector = UVC_PU_BRIGHTNESS_CONTROL;
	_brightCtrl.intendedSize = 2;
	_brightCtrl.hasMin = YES;
	_brightCtrl.hasMax = YES;
	_brightCtrl.hasDef = YES;
	_brightCtrl.isSigned = YES;
	_brightCtrl.isRelative = NO;
	_contrastCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_contrastCtrl.selector = UVC_PU_CONTRAST_CONTROL;
	_contrastCtrl.intendedSize = 2;
	_contrastCtrl.hasMin = YES;
	_contrastCtrl.hasMax = YES;
	_contrastCtrl.hasDef = YES;
	_contrastCtrl.isSigned = NO;
	_contrastCtrl.isRelative = NO;
	_gainCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_gainCtrl.selector = UVC_PU_GAIN_CONTROL;
	_gainCtrl.intendedSize = 2;
	_gainCtrl.hasMin = YES;
	_gainCtrl.hasMax = YES;
	_gainCtrl.hasDef = YES;
	_gainCtrl.isSigned = NO;
	_gainCtrl.isRelative = NO;
	_powerLineCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_powerLineCtrl.selector = UVC_PU_POWER_LINE_FREQUENCY_CONTROL;
	_powerLineCtrl.intendedSize = 2;
	_powerLineCtrl.hasMin = YES;
	_powerLineCtrl.hasMax = YES;
	_powerLineCtrl.hasDef = YES;
	_powerLineCtrl.isSigned = NO;
	_powerLineCtrl.isRelative = NO;
	_autoHueCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_autoHueCtrl.selector = UVC_PU_HUE_AUTO_CONTROL;
	_autoHueCtrl.intendedSize = 2;
	_autoHueCtrl.hasMin = NO;
	_autoHueCtrl.hasMax = NO;
	_autoHueCtrl.hasDef = YES;
	_autoHueCtrl.isSigned = NO;
	_autoHueCtrl.isRelative = NO;
	_hueCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_hueCtrl.selector = UVC_PU_HUE_CONTROL;
	_hueCtrl.intendedSize = 2;
	_hueCtrl.hasMin = YES;
	_hueCtrl.hasMax = YES;
	_hueCtrl.hasDef = YES;
	_hueCtrl.isSigned = YES;
	_hueCtrl.isRelative = NO;
	_saturationCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_saturationCtrl.selector = UVC_PU_SATURATION_CONTROL;
	_saturationCtrl.intendedSize = 2;
	_saturationCtrl.hasMin = YES;
	_saturationCtrl.hasMax = YES;
	_saturationCtrl.hasDef = YES;
	_saturationCtrl.isSigned = NO;
	_saturationCtrl.isRelative = NO;
	_sharpnessCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_sharpnessCtrl.selector = UVC_PU_SHARPNESS_CONTROL;
	_sharpnessCtrl.intendedSize = 2;
	_sharpnessCtrl.hasMin = YES;
	_sharpnessCtrl.hasMax = YES;
	_sharpnessCtrl.hasDef = YES;
	_sharpnessCtrl.isSigned = NO;
	_sharpnessCtrl.isRelative = NO;
	_gammaCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_gammaCtrl.selector = UVC_PU_GAMMA_CONTROL;
	_gammaCtrl.intendedSize = 2;
	_gammaCtrl.hasMin = YES;
	_gammaCtrl.hasMax = YES;
	_gammaCtrl.hasDef = YES;
	_gammaCtrl.isSigned = NO;
	_gammaCtrl.isRelative = NO;
	_whiteBalanceAutoTempCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_whiteBalanceAutoTempCtrl.selector = UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL;
	_whiteBalanceAutoTempCtrl.intendedSize = 1;
	_whiteBalanceAutoTempCtrl.hasMin = NO;
	_whiteBalanceAutoTempCtrl.hasMax = NO;
	_whiteBalanceAutoTempCtrl.hasDef = YES;
	_whiteBalanceAutoTempCtrl.isSigned = NO;
	_whiteBalanceAutoTempCtrl.isRelative = NO;
	_whiteBalanceTempCtrl.unit = UVC_PROCESSING_UNIT_ID;
	_whiteBalanceTempCtrl.selector = UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL;
	_whiteBalanceTempCtrl.intendedSize = 4;			//	WARNING: the spec says this should only have a length of "2", but it throws errors unless i use a length of 4!
	_whiteBalanceTempCtrl.hasMin = YES;
	_whiteBalanceTempCtrl.hasMax = YES;
	_whiteBalanceTempCtrl.hasDef = YES;
	_whiteBalanceTempCtrl.isSigned = NO;
	_whiteBalanceTempCtrl.isRelative = NO;
}


/*===================================================================================*/
#pragma mark --------------------- init/dealloc
/*------------------------------------*/


/*
- (id) initWithQTCaptureDevice:(QTCaptureDevice *)dev	{
	//NSLog(@"%s",__func__);
	if (dev != nil)	{
		return [self initWithDeviceIDString:[dev uniqueID]];
	}
	[self release];
	return nil;
}
*/
- (id) initWithDeviceIDString:(NSString *)n	{
	//NSLog(@"%s ... %@",__func__,n);
	if (n != nil)	{
		unsigned int locationID = 0;
		sscanf([n UTF8String], "0x%8x",&locationID);
		if (locationID) return [self initWithLocationID:locationID];
	}
	[self release];
	return nil;
}
- (id) initWithLocationID:(UInt32)locationID {
	//NSLog(@"%s ... %d, %X",__func__,(unsigned int)locationID,(unsigned int)locationID);
	self = [super init];
	if (self!=nil) {
		//	technically i don't need to set these here- they're calculated below from the BusProber, but default values are good, m'kay?
		inputTerminalID = 1;
		processingUnitID = 2;	//	logitech C910
		//processingUnitID = 4;	//	works for microsoft lifecam!
		//processingUnitID = 3;	//	works for "FaceTime HD Camera" on gen-8 macbook pros!
		deviceLocationID = locationID;
		interface = NULL;
		
		//	first of all, i need to harvest a couple pieces of data from the USB device- i need:
		//		- the "Terminal ID" of the "VDC ((Control) Input Terminal" of the "Video/Control" interface in the "Configuration Descriptor"
		//		- the "Unit ID" of the "VDC ((Control) Processing Unit" of the "Video/Control" interface in the "Configuration Descriptor"
		BusProber		*prober = [[BusProber alloc] init];
		NSMutableArray	*devices = [prober devicesArray];
		for (BusProbeDevice *devicePtr in devices)	{
			if ([devicePtr locationID] == locationID)	{
				//NSLog(@"\t\tfound device %@",[devicePtr deviceName]);
				NSDictionary		*tmpDict = [devicePtr dictionaryVersionOfMe];
				//NSLog(@"\t\ttop-level keys are %@",[tmpDict allKeys]);
				//NSLog(@"\t\tdevice dict is %@",tmpDict);
				//[tmpDict writeToFile:[@"~/Desktop/tmpOut.plist" stringByExpandingTildeInPath] atomically:YES];
				
				
				//	get the dict at the key 'nodeData'
				NSDictionary		*topLevelNodeDataDict = (tmpDict==nil) ? nil : [tmpDict objectForKey:@"nodeData"];
				//	from the node data dict, get the 'children' array
				NSArray				*topLevelNodeChildren = (topLevelNodeDataDict==nil) ? nil : [topLevelNodeDataDict objectForKey:@"children"];
				//	run through the children- each child is a dict, look for the dict with a "nodeName" that contains the string "Configuration Descriptor"
				for (NSDictionary *topLevelNodeChild in topLevelNodeChildren)	{
					NSString		*topLevelNodeChildName = [topLevelNodeChild objectForKey:@"nodeName"];
					if ([topLevelNodeChildName containsString:@"Configuration Descriptor"])	{
						
						//	get the 'children' array from the top level node child dict
						NSArray			*configDescriptorChildren = [topLevelNodeChild objectForKey:@"children"];
						//	run through the children- each child is a dict, look for the dict with a "nodeName" that contains the string "Video/Control"
						for (NSDictionary *configDescriptorChild in configDescriptorChildren)	{
							NSString		*configChildName = [configDescriptorChild objectForKey:@"nodeName"];
							if ([configChildName containsString:@"Video/Control"])	{
								
								//	get the 'children' array from the config descriptor child dict
								NSArray			*videoControlChildren = [configDescriptorChild objectForKey:@"children"];
								//	run through the children- each child is a dict, look for the dict with a "nodeName" that contains the string "VDC (Control) Input Terminal"
								for (NSDictionary *videoControlChild in videoControlChildren)	{
									NSString		*controlChildName = [videoControlChild objectForKey:@"nodeName"];
									if ([controlChildName containsString:@"VDC (Control) Input Terminal"])	{
										
										//	get the 'children' array from the control child dict
										NSArray			*inputTerminalChildren = [videoControlChild objectForKey:@"children"];
										//	run through the children- each child is a dict, look for the child with a string at the key "Terminal ID"
										for (NSDictionary *inputTerminalChild in inputTerminalChildren)	{
											NSString		*terminalIDString = [inputTerminalChild objectForKey:@"Terminal ID"];
											if (terminalIDString != nil)	{
												//NSLog(@"\t\tterminalIDString is %@",terminalIDString);
												inputTerminalID = (int)[terminalIDString integerValue];
												//NSLog(@"\t\tverifying: inputTerminalID now %d",inputTerminalID);
												
												//	...i can break because i found the terminal ID
												break;
											}
										}
										
										//	...i can break because i found the input terminal dict
										break;
									}
								}
								//	run through the children- each child is a dict, look for the dict with a "nodeName" that contains the string "VDC (Control) Processing Unit"
								for (NSDictionary *videoControlChild in videoControlChildren)	{
									NSString		*controlChildName = [videoControlChild objectForKey:@"nodeName"];
									if ([controlChildName containsString:@"VDC (Control) Processing Unit"])	{
										
										//	get the 'children' array from the control child dict
										NSArray			*processingUnitChildren = [videoControlChild objectForKey:@"children"];
										//	run through the children- each child is a dict, look for the child with a string at the key "Unit ID:"
										for (NSDictionary *processingUnitChild in processingUnitChildren)	{
											NSString		*unitIDString = [processingUnitChild objectForKey:@"Unit ID:"];
											if (unitIDString != nil)	{
												//NSLog(@"\t\tunitIDString is %@",unitIDString);
												processingUnitID = (int)[unitIDString integerValue];
												//NSLog(@"\t\tverifying: processingUnitID now %d",processingUnitID);
												
												//	...i can break because i found the unit ID
												break;
											}
										}
										
										//	...i can break because i found the processing unit dict
										break;
									}
								}
								//	...i can break because i found the video/control dict
								break;
							}
						}
						//	...i can break because i found the configuration descriptor dict
						break;
					}
				}
				
				
				
				break;
			}
		}
		if (prober != nil)	{
			[prober release];
			prober = nil;
		}
		
		
		
		//	Find All USB Devices, get their locationId and check if it matches the requested one
		CFMutableDictionaryRef		matchingDict = IOServiceMatching(kIOUSBDeviceClassName);
		io_iterator_t				serviceIterator;
		
		IOServiceGetMatchingServices( kIOMasterPortDefault, matchingDict, &serviceIterator );
		
		BOOL						successfullInit = NO;
		io_service_t				camera;
		while( (camera = IOIteratorNext(serviceIterator)) ) {
			// Get DeviceInterface
			IOUSBDeviceInterface	**deviceInterface = NULL;
			IOCFPlugInInterface		**plugInInterface = NULL;
			SInt32					score;
			kern_return_t			kr;
			
			kr = IOCreatePlugInInterfaceForService( camera, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugInInterface, &score );
			if( (kIOReturnSuccess != kr) || !plugInInterface ) {
				NSLog( @"CameraControl Error: IOCreatePlugInInterfaceForService returned 0x%08x.", kr );
				if (plugInInterface!=NULL)	{
					IODestroyPlugInInterface(plugInInterface);
					plugInInterface = NULL;
					break;
				}
			}
			else	{
				HRESULT					res = (*plugInInterface)->QueryInterface(plugInInterface, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID), (LPVOID*) &deviceInterface );
				(*plugInInterface)->Release(plugInInterface);
				if( res || deviceInterface == NULL ) {
					NSLog( @"CameraControl Error: QueryInterface returned %d.\n", (int)res );
					//	clean up the plugin interface
					if (plugInInterface!=NULL)	{
						IODestroyPlugInInterface(plugInInterface);
						plugInInterface = NULL;
						break;
					}
				}
				else	{
					UInt32 currentLocationID = 0;
					(*deviceInterface)->GetLocationID(deviceInterface, &currentLocationID);
					//	if this is the USB device i was looking for...
					if( currentLocationID == locationID ) {
						//	get the usb interface
						interface = [self _getControlInferaceWithDeviceInterface:deviceInterface];
						[self generalInit];
						successfullInit = YES;
						//	clean up the plugin interface
						if (plugInInterface!=NULL)	{
							IODestroyPlugInInterface(plugInInterface);
							plugInInterface = NULL;
						}
						break;
					}
					//	clean up the plugin interface
					if (plugInInterface!=NULL)	{
						IODestroyPlugInInterface(plugInInterface);
						plugInInterface = NULL;
					}
				}
			}
			
		} // end while
		
		
		//	if i successfully init'ed the camera, i can return myself
		if (successfullInit)
			return self;
		//	else i couldn't successfully init myself, something went wrong/i couldn't connect: release self and return nil;
		NSLog(@"\t\tERR: couldn't create VVUVCController with locationID %d, %X",(unsigned int)locationID,(unsigned int)locationID);
		[self release];
		return nil;
	}
	return self;
}
- (IOUSBInterfaceInterface190 **) _getControlInferaceWithDeviceInterface:(IOUSBDeviceInterface **)deviceInterface {
	//NSLog(@"%s",__func__);
	io_iterator_t					interfaceIterator;
	IOUSBFindInterfaceRequest		interfaceRequest;
	
	interfaceRequest.bInterfaceClass = UVC_CONTROL_INTERFACE_CLASS;
	interfaceRequest.bInterfaceSubClass = UVC_CONTROL_INTERFACE_SUBCLASS;
	interfaceRequest.bInterfaceProtocol = kIOUSBFindInterfaceDontCare;
	interfaceRequest.bAlternateSetting = kIOUSBFindInterfaceDontCare;
	
	IOReturn success = (*deviceInterface)->CreateInterfaceIterator( deviceInterface, &interfaceRequest, &interfaceIterator );
	if( success != kIOReturnSuccess ) {
		return NULL;
	}
	
	io_service_t		ioDeviceObj;
	HRESULT				result;
	
	if( (ioDeviceObj = IOIteratorNext(interfaceIterator)) ) {
		IOCFPlugInInterface				**ioPlugin = NULL;
		IOUSBInterfaceInterface190		**controlInterface;
		//IOUSBDeviceRef					deviceInterface = NULL;
		//	Create an intermediate plug-in
		SInt32						score;
		kern_return_t				kr;
		
		kr = IOCreatePlugInInterfaceForService( ioDeviceObj, kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID, &ioPlugin, &score );
		
		//	Release the ioDeviceObj object after getting the plug-in
		kr = IOObjectRelease(ioDeviceObj);
		if( (kr != kIOReturnSuccess) || !ioPlugin ) {
			NSLog( @"CameraControl Error: Unable to create a plug-in (%08x)\n", kr );
			return NULL;
		}
		
		
		//	Now create the device interface for the interface
		result = (*ioPlugin)->QueryInterface( ioPlugin, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID), (LPVOID *) &controlInterface );
		//	No longer need the intermediate plug-in
		(*ioPlugin)->Release(ioPlugin);
		
		if (result || !controlInterface) {
			NSLog( @"CameraControl Error: Couldn’t create a control interface for the interface (%08x)", (int) result );
			return NULL;
		}
		
		return controlInterface;
	}
	
	return NULL;
}
- (void) generalInit	{
	
	//delegate = nil;
	
	scanningMode.ctrlInfo = &_scanCtrl;
	autoExposureMode.ctrlInfo = &_autoExposureModeCtrl;
	autoExposurePriority.ctrlInfo = &_autoExposurePriorityCtrl;
	exposureTime.ctrlInfo = &_exposureTimeCtrl;
	iris.ctrlInfo = &_irisCtrl;
	autoFocus.ctrlInfo = &_autoFocusCtrl;
	focus.ctrlInfo = &_focusCtrl;
	zoom.ctrlInfo = &_zoomCtrl;
	panTilt.ctrlInfo = &_panTiltCtrl;
	panTiltRel.ctrlInfo = &_panTiltRelCtrl;
	roll.ctrlInfo = &_rollCtrl;
	rollRel.ctrlInfo = &_rollRelCtrl;
	
	backlight.ctrlInfo = &_backlightCtrl;
	bright.ctrlInfo = &_brightCtrl;
	contrast.ctrlInfo = &_contrastCtrl;
	gain.ctrlInfo = &_gainCtrl;
	powerLine.ctrlInfo = &_powerLineCtrl;
	autoHue.ctrlInfo = &_autoHueCtrl;
	hue.ctrlInfo = &_hueCtrl;
	saturation.ctrlInfo = &_saturationCtrl;
	sharpness.ctrlInfo = &_sharpnessCtrl;
	gamma.ctrlInfo = &_gammaCtrl;
	autoWhiteBalance.ctrlInfo = &_whiteBalanceAutoTempCtrl;
	whiteBalance.ctrlInfo = &_whiteBalanceTempCtrl;
	
	if (interface)	{
		(*interface)->GetInterfaceNumber(interface,&interfaceNumber);
	}
	
	[self _populateAllParams];
	
	//	create the nib from my class name
	theNib = [[NSNib alloc] initWithNibNamed:[self className] bundle:[NSBundle bundleForClass:[self class]]];
	//	unpack the nib, instantiating the object
	[theNib instantiateWithOwner:self topLevelObjects:&nibTopLevelObjects];
	//	retain the array of top-level objects (they have to be explicitly freed later)
	[nibTopLevelObjects retain];
	
	if (uiCtrlr != nil)
		[uiCtrlr _pushCameraControlStateToUI];
}
- (void) dealloc {
	//NSLog(@"%s ... %p",__func__,self);
	[self closeSettingsWindow];
	
	if( interface ) {
		(*interface)->USBInterfaceClose(interface);
		(*interface)->Release(interface);
	}
	
	//	free this (i retained it explicitly earlier)
	if (nibTopLevelObjects != nil)	{
		[nibTopLevelObjects release];
		nibTopLevelObjects = nil;
	}
	//	release the nib
	if (theNib != nil)	{
		[theNib release];
		theNib = nil;
	}
	[super dealloc];
}


/*===================================================================================*/
#pragma mark --------------------- saving/restoring state
/*------------------------------------*/


- (NSMutableDictionary *) createSnapshot	{
	NSMutableDictionary		*returnMe = [NSMutableDictionary dictionaryWithCapacity:0];
	
	[returnMe setObject:[NSNumber numberWithBool:[self interlaced]] forKey:@"interlaced"];
	[returnMe setObject:[NSNumber numberWithInt:[self autoExposureMode]] forKey:@"autoExposureMode"];
	[returnMe setObject:[NSNumber numberWithBool:[self autoExposurePriority]] forKey:@"autoExposurePriority"];
	[returnMe setObject:[NSNumber numberWithLong:[self exposureTime]] forKey:@"exposureTime"];
	[returnMe setObject:[NSNumber numberWithLong:[self iris]] forKey:@"iris"];
	[returnMe setObject:[NSNumber numberWithBool:[self autoFocus]] forKey:@"autoFocus"];
	[returnMe setObject:[NSNumber numberWithLong:[self focus]] forKey:@"focus"];
	[returnMe setObject:[NSNumber numberWithLong:[self zoom]] forKey:@"zoom"];
	//[returnMe setObject:[NSNumber numberWithLong:[self pan]] forKey:@"pan"];
	//[returnMe setObject:[NSNumber numberWithLong:[self tilt]] forKey:@"tilt"];
	//[returnMe setObject:[NSNumber numberWithLong:[self roll]] forKey:@"roll"];
	[returnMe setObject:[NSNumber numberWithLong:[self backlight]] forKey:@"backlight"];
	[returnMe setObject:[NSNumber numberWithLong:[self bright]] forKey:@"bright"];
	[returnMe setObject:[NSNumber numberWithLong:[self contrast]] forKey:@"contrast"];
	[returnMe setObject:[NSNumber numberWithLong:[self gain]] forKey:@"gain"];
	[returnMe setObject:[NSNumber numberWithLong:[self powerLine]] forKey:@"powerLine"];
	[returnMe setObject:[NSNumber numberWithBool:[self autoHue]] forKey:@"autoHue"];
	[returnMe setObject:[NSNumber numberWithLong:[self hue]] forKey:@"hue"];
	[returnMe setObject:[NSNumber numberWithLong:[self saturation]] forKey:@"saturation"];
	[returnMe setObject:[NSNumber numberWithLong:[self sharpness]] forKey:@"sharpness"];
	[returnMe setObject:[NSNumber numberWithLong:[self gamma]] forKey:@"gamma"];
	[returnMe setObject:[NSNumber numberWithBool:[self autoWhiteBalance]] forKey:@"autoWhiteBalance"];
	[returnMe setObject:[NSNumber numberWithLong:[self whiteBalance]] forKey:@"whiteBalance"];
	return returnMe;
}
- (void) loadSnapshot:(NSDictionary *)s	{
	//NSLog(@"%s",__func__);
	//NSLog(@"\t\t%@",s);
	if (s == nil)
		return;
	
	NSNumber		*tmpNum = nil;
	BOOL			needsToRepopulate = NO;
	
	//	if i have to repopulate the params, do so now!
	if (needsToRepopulate)	{
		//NSLog(@"\t\trepopulating params in %s, input/processing ID changed!",__func__);
		[self _populateAllParams];
	}
	
	//	reset all the params to their defaults, or the changes won't "take" on some cameras!
	[self resetParamsToDefaults];
	
	//	proceed with loading the rest of the snap....
	tmpNum = [s objectForKey:@"interlaced"];
	if (tmpNum!=nil)	{
		[self setInterlaced:scanningMode.def];
		[self setInterlaced:[tmpNum boolValue]];
	}
	
	tmpNum = [s objectForKey:@"autoExposureMode"];
	if (tmpNum!=nil)	{
		[self setAutoExposureMode:[tmpNum intValue]];
	}
	
	tmpNum = [s objectForKey:@"autoExposurePriority"];
	if (tmpNum!=nil)	{
		[self setAutoExposurePriority:[tmpNum boolValue]];
	}
	
	tmpNum = [s objectForKey:@"exposureTime"];
	if (tmpNum!=nil)	{
		[self setExposureTime:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"iris"];
	if (tmpNum!=nil)	{
		[self setIris:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"autoFocus"];
	if (tmpNum!=nil)	{
		[self setAutoFocus:[tmpNum boolValue]];
	}
	
	tmpNum = [s objectForKey:@"focus"];
	if (tmpNum!=nil)	{
		[self setFocus:focus.def];
		[self setFocus:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"zoom"];
	if (tmpNum!=nil)	{
		[self setZoom:[tmpNum longValue]];
	}
	/*
	tmpNum = [s objectForKey:@"pan"];
	if (tmpNum!=nil)
		[self setPan:[tmpNum longValue]];
	tmpNum = [s objectForKey:@"tilt"];
	if (tmpNum!=nil)
		[self setTilt:[tmpNum longValue]];
	tmpNum = [s objectForKey:@"roll"];
	if (tmpNum!=nil)
		[self setRoll:[tmpNum longValue]];
	*/
	tmpNum = [s objectForKey:@"backlight"];
	if (tmpNum != nil)	{
		[self setBacklight:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"bright"];
	if (tmpNum!=nil)	{
		[self setBright:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"contrast"];
	if (tmpNum!=nil)	{
		[self setContrast:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"gain"];
	if (tmpNum!=nil)	{
		[self setGain:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"powerLine"];
	if (tmpNum!=nil)	{
		[self setPowerLine:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"autoHue"];
	if (tmpNum!=nil)	{
		[self setAutoHue:[tmpNum boolValue]];
	}
	
	tmpNum = [s objectForKey:@"hue"];
	if (tmpNum!=nil)	{
		[self setHue:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"saturation"];
	if (tmpNum!=nil)	{
		[self setSaturation:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"sharpness"];
	if (tmpNum!=nil)	{
		[self setSharpness:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"gamma"];
	if (tmpNum!=nil)	{
		[self setGamma:[tmpNum longValue]];
	}
	
	tmpNum = [s objectForKey:@"autoWhiteBalance"];
	if (tmpNum!=nil)	{
		[self setAutoWhiteBalance:[tmpNum boolValue]];
	}
	
	tmpNum = [s objectForKey:@"whiteBalance"];
	if (tmpNum!=nil)	{
		[self setWhiteBalance:[tmpNum longValue]];
	}
	
	
	if (uiCtrlr != nil)
		[uiCtrlr _pushCameraControlStateToUI];
	
	//NSLog(@"\t\t%s - FINISHED",__func__);
}


/*===================================================================================*/
#pragma mark --------------------- backend
/*------------------------------------*/


//	this method gets called by _setData and _getDataFor
- (BOOL) _sendControlRequest:(IOUSBDevRequest *)controlRequest {
	//NSLog(@"%s",__func__);
	//NSLog(@"\t\tindex=%d",controlRequest->wIndex);
	//NSLog(@"\t\trequestType=%d",controlRequest->bmRequestType);
	//NSLog(@"\t\trequest=%d",controlRequest->bRequest);
	//NSLog(@"\t\t0x%X 0x%X 0x%X 0x%X",controlRequest.bRequest,controlRequest.wValue,controlRequest.wIndex,controlRequest.wLength);
	if( !interface ){
		NSLog( @"CameraControl Error: No interface to send request" );
		return NO;
	}
	/*
	//Now open the interface. This will cause the pipes associated with
	//the endpoints in the interface descriptor to be instantiated
	kern_return_t kr = (*interface)->USBInterfaceOpen(interface);
	if (kr != kIOReturnSuccess)	{
		NSLog( @"CameraControl Error: Unable to open interface (%08x)\n", kr );
		return NO;
	}
	kr = (*interface)->ControlRequest( interface, 0, &controlRequest );
	*/
	kern_return_t kr = (*interface)->ControlRequest( interface, 0, controlRequest );
	if( kr != kIOReturnSuccess ) {
		kr = (*interface)->USBInterfaceClose(interface);
		//NSLog( @"CameraControl Error: Control request failed: %08x", kr );
		return NO;
	}
	/*
	kr = (*interface)->USBInterfaceClose(interface);
	*/
	return YES;
}
- (int) _requestValType:(int)requestType forControl:(const uvc_control_info_t *)ctrl returnVal:(void **)ret	{
	//NSLog(@"%s ... 0x%X",__func__,requestType);
	int					returnMe = 0;
	IOUSBDevRequest		controlRequest;
	controlRequest.bmRequestType = USBmakebmRequestType( kUSBIn, kUSBClass, kUSBInterface );
	controlRequest.bRequest = requestType;
	controlRequest.wValue = (ctrl->selector << 8) | 0x00;
	//NSLog(@"\t\tctrl->unit is %d",ctrl->unit);
	//NSLog(@"\t\tctrl->unit << 8 is %d",ctrl->unit << 8);
	controlRequest.wIndex = (ctrl->unit==UVC_INPUT_TERMINAL_ID) ? inputTerminalID : processingUnitID;
	controlRequest.wIndex = ((controlRequest.wIndex<<8) | interfaceNumber);
	//controlRequest.wIndex = ((ctrl->unit << 8) | interfaceNumber);
	//controlRequest.wIndex = (512 | interfaceNumber);
	//	if it's a "get info" request, the length is always going to be 1!
	controlRequest.wLength = (requestType==UVC_GET_INFO) ? 1 : ctrl->intendedSize;
	controlRequest.wLenDone = 0;
	
	*ret = malloc(controlRequest.wLength);
	bzero(*ret,controlRequest.wLength);
	controlRequest.pData = *ret;
	
	//	send the request, if it wasn't successful, return -1
	if (![self _sendControlRequest:&controlRequest])
		returnMe = -1;
	//	else returnMe should be the number of bytes i actually read!
	else
		returnMe = controlRequest.wLenDone;
	
	//	if the number of bytes i actually read was <= 0 or the request failed, free the buffer i allocated!
	if (returnMe <= 0)	{
		free(*ret);
		*ret = nil;
		controlRequest.pData = nil;
	}
	
	return returnMe;
}
- (BOOL) _setBytes:(void *)bytes sized:(int)size toControl:(const uvc_control_info_t *)ctrl	{
	//NSLog(@"%s",__func__);
	BOOL			returnMe = NO;
	long			tmpLong = 0x00000000;
	memcpy(&tmpLong,bytes,size);
	//NSLog(@"\t\tbytes are %ld, size is %d",tmpLong,size);
	IOUSBDevRequest		controlRequest;
	controlRequest.bmRequestType = USBmakebmRequestType( kUSBOut, kUSBClass, kUSBInterface );
	controlRequest.bRequest = UVC_SET_CUR;
	controlRequest.wValue = (ctrl->selector << 8) | 0x00;
	controlRequest.wIndex = (ctrl->unit==UVC_INPUT_TERMINAL_ID) ? inputTerminalID : processingUnitID;
	controlRequest.wIndex = ((controlRequest.wIndex<<8) | interfaceNumber);
	//controlRequest.wIndex = ((ctrl->unit << 8) | interfaceNumber);
	//controlRequest.wIndex = (512 | interfaceNumber);
	controlRequest.wLength = size;
	controlRequest.wLenDone = 0;
	controlRequest.pData = bytes;
	returnMe = [self _sendControlRequest:&controlRequest];
	return returnMe;
}
- (void) _populateAllParams	{
	
	[self _populateParam:&scanningMode];
	[self _populateParam:&autoExposureMode];
	[self _populateParam:&autoExposurePriority];
	[self _populateParam:&exposureTime];
	[self _populateParam:&iris];
	[self _populateParam:&autoFocus];
	[self _populateParam:&focus];
	[self _populateParam:&zoom];
	[self _populateParam:&panTilt];
	[self _populateParam:&panTiltRel];
	[self _populateParam:&roll];
	[self _populateParam:&rollRel];
	
	[self _populateParam:&backlight];
	[self _populateParam:&bright];
	[self _populateParam:&contrast];
	[self _populateParam:&gain];
	[self _populateParam:&powerLine];
	[self _populateParam:&autoHue];
	[self _populateParam:&hue];
	[self _populateParam:&saturation];
	[self _populateParam:&sharpness];
	[self _populateParam:&gamma];
	
	[self _populateParam:&autoWhiteBalance];
	
	[self _populateParam:&whiteBalance];
	/*
	NSLog(@"\t\t*******************");
	NSLog(@"\t\t (min) - [val] - (max), def");
	NSLog(@"\t\t*******************");
	NSLogParam(@"\t\t scanning",scanningMode);
	NSLogParam(@"\t\t auto exp mode",autoExposureMode);
	NSLogParam(@"\t\t auto exp priority",autoExposurePriority);
	NSLogParam(@"\t\t exposure time",exposureTime);
	NSLogParam(@"\t\t iris",iris);
	NSLogParam(@"\t\t auto focus",autoFocus);
	NSLogParam(@"\t\t focus",focus);
	NSLogParam(@"\t\t zoom",zoom);
	NSLogParam(@"\t\t pan/tilt (abs)",panTilt);
	NSLogParam(@"\t\t pan/tilt (rel)",panTiltRel);
	NSLogParam(@"\t\t roll (abs)",roll);
	NSLogParam(@"\t\t roll (rel)",rollRel);
	
	NSLogParam(@"\t\t backlight",backlight);
	NSLogParam(@"\t\t bright",bright);
	NSLogParam(@"\t\t contrast",contrast);
	NSLogParam(@"\t\t gain",gain);
	NSLogParam(@"\t\t power",powerLine);
	NSLogParam(@"\t\t auto hue",autoHue);
	NSLogParam(@"\t\t hue",hue);
	NSLogParam(@"\t\t sat",saturation);
	NSLogParam(@"\t\t sharp",sharpness);
	NSLogParam(@"\t\t gamma",gamma);
	NSLogParam(@"\t\t auto wb",autoWhiteBalance);
	NSLogParam(@"\t\t wb",whiteBalance);
	NSLog(@"\t\t*******************");
	*/
}
- (void) _populateParam:(uvc_param *)param	{
	//NSLog(@"%s ... %p",__func__,param);
	/*
	if (param == &panTilt || param == &roll)	{
		NSLog(@"\t\terr: pan/tilt and roll temporarily disabled! %s",__func__);
		goto DISABLED_PARAM;
	}
	*/
	//long			*longPtr = nil;
	void			*bytesPtr = nil;
	long			tmpLong = 0;
	int				bytesRead = 0;
	
	//	do a "get info" request on the param first to check the param and make sure it can be set
	//bytesRead = [self _requestValType:UVC_GET_INFO forControl:param->ctrlInfo returnVal:(void **)&longPtr];
	bytesRead = [self _requestValType:UVC_GET_INFO forControl:param->ctrlInfo returnVal:&bytesPtr];
	if (bytesRead <= 0)	{
		//NSLog(@"\t\terr: couldn't get info %s",__func__);
		goto DISABLED_PARAM;
	}
	tmpLong = 0x00000000;
	memcpy(&tmpLong,bytesPtr,bytesRead);
	free(bytesPtr);
	bytesPtr = nil;
	
	//	make sure the param can be get and set- otherwise, disable the param!
	BOOL			canGetAndSet = (((tmpLong & 0x01) == 0x01) && ((tmpLong & 0x02) == 0x02)) ? YES : NO;
	if (!canGetAndSet)	{
		//NSLog(@"\t\terr: can't get or set %s",__func__);
		goto DISABLED_PARAM;
	}
	
	//	if i'm here, the "get info" request was successful, and the param can be both set and retrieved!
	
	param->supported = YES;
	
	//	the size of the returned val will change, so make sure to mask out the irrelevant bits
	int			paramSize = param->ctrlInfo->intendedSize;
	long		valSizeMask;
	if (paramSize == 1)
		valSizeMask = 0x00FF;
	else if (paramSize == 2)
		valSizeMask = 0xFFFF;
	else if (paramSize == 4)
		valSizeMask = 0xFFFFFFFF;
	else if (paramSize > 4)	{
		//NSLog(@"\t\terr: paramSize is %d, must be handled differently! %s",paramSize,__func__);
		goto DISABLED_PARAM;
	}
	//	figure out how far i have to shift the bits to find the "negative" bit if the value is signed!
	int					shiftToGetSignBit = ((paramSize * 8) - 1);
	//	calculate the mask to reveal the sign, and the mask to remove the sign from the value!
	unsigned long		maskToRevealSign = (param->ctrlInfo->isSigned) ? (0x0001 << shiftToGetSignBit) : (0x0000);
	unsigned long		maskToRemoveSign = maskToRevealSign-1;
	//NSLog(@"\t\tshift to get sign bit is %d",shiftToGetSignBit);
	//NSLog(@"\t\tmask to reveal sign is %ld",maskToRevealSign);
	//NSLog(@"\t\tmask to remove sign is %ld",maskToRemoveSign);
	
	//	get the current val (which is definitely supported)
	bytesRead = [self _requestValType:UVC_GET_CUR forControl:param->ctrlInfo returnVal:&bytesPtr];
	if (bytesRead <= 0)	{
		//NSLog(@"\t\terr: couldn't get current val in %s",__func__);
		goto DISABLED_PARAM;
	}
	tmpLong = 0x00000000;
	memcpy(&tmpLong,bytesPtr,bytesRead);
	free(bytesPtr);
	bytesPtr = nil;
	param->val = (tmpLong & valSizeMask & maskToRemoveSign);
	//NSLog(@"\t\traw val is %ld, refined val is %ld",tmpLong,param->val);
	param->actualSize = bytesRead;
	//if (param->actualSize != param->ctrlInfo->intendedSize)
	//	NSLog(@"\t\t****** err: actual size doesn't match intended size!");
	if ((tmpLong & maskToRevealSign) != 0)
		param->val = param->val * -1;
	
	//	min
	if (param->ctrlInfo->hasMin)	{
		bytesRead = [self _requestValType:UVC_GET_MIN forControl:param->ctrlInfo returnVal:&bytesPtr];
		if (bytesRead <= 0)
			goto DISABLED_PARAM;
		tmpLong = 0x00000000;
		memcpy(&tmpLong,bytesPtr,bytesRead);
		free(bytesPtr);
		bytesPtr = nil;
		if ((tmpLong & maskToRevealSign) == 0)
			param->min = (tmpLong & valSizeMask & maskToRemoveSign);
		else
			param->min = -((~tmpLong & valSizeMask) + 1);
		//NSLog(@"\t\traw min is %ld, refined min is %ld",tmpLong,param->min);
		//if (param->actualSize != bytesRead)
		//	NSLog(@"******* ERR: bytes read on min don't match bytes read on val!");
	}
	//	max
	if (param->ctrlInfo->hasMax)	{
		bytesRead = [self _requestValType:UVC_GET_MAX forControl:param->ctrlInfo returnVal:&bytesPtr];
		if (bytesRead <= 0)
			goto DISABLED_PARAM;
		tmpLong = 0x00000000;
		memcpy(&tmpLong,bytesPtr,bytesRead);
		free(bytesPtr);
		bytesPtr = nil;
		if ((tmpLong & maskToRevealSign) == 0)
			param->max = (tmpLong & valSizeMask & maskToRemoveSign);
		else
			param->max = -((~tmpLong & valSizeMask) + 1);
		//NSLog(@"\t\traw max is %ld, refined max is %ld",tmpLong,param->max);
		//if (param->actualSize != bytesRead)
		//	NSLog(@"******* ERR: bytes read on max don't match bytes read on val!");
	}
	//	default
	if (param->ctrlInfo->hasDef)	{
		bytesRead = [self _requestValType:UVC_GET_DEF forControl:param->ctrlInfo returnVal:&bytesPtr];
		if (bytesRead <= 0)
			goto DISABLED_PARAM;
		tmpLong = 0x00000000;
		memcpy(&tmpLong,bytesPtr,bytesRead);
		free(bytesPtr);
		bytesPtr = nil;
		if ((tmpLong & maskToRevealSign) == 0)
			param->def = (tmpLong & valSizeMask & maskToRemoveSign);
		else
			param->def = -((~tmpLong & valSizeMask) + 1);
		//NSLog(@"\t\traw def is %ld, refined def is %ld",tmpLong,param->def);
		//if (param->actualSize != bytesRead)
		//	NSLog(@"******* ERR: bytes read on default don't match bytes read on val!");
	}
	
	return;
	DISABLED_PARAM:
	//NSLog(@"\t\tDISABLED_PARAM %s",__func__);
	param->supported = NO;
	param->min = -1;
	param->max = -1;
	param->val = -1;
	param->def = -1;
	param->actualSize = -1;
	//param->ctrlInfo = nil;
}
- (BOOL) _pushParamToDevice:(uvc_param *)param	{
	//NSLog(@"%s",__func__);
	if (param == nil)
		return NO;
	/*
	if (param == &panTilt || param == &roll)	{
		NSLog(@"\t\terr: pan/tilt and roll temporarily disabled!");
		return;
	}
	*/
	//int			paramSize = param->ctrlInfo->intendedSize;
	int			paramSize = param->actualSize;
	if (paramSize <= 0)
		return NO;
	
	BOOL		returnMe = NO;
	
	int			valToSend = 0x0000;
	//	if the val may be signed, i may have to assemble the value to be sent manually
	if (param->ctrlInfo->isSigned)	{
		//NSLog(@"\t\traw val is %ld",param->val);
		valToSend = (int)labs(param->val);
		//NSLog(@"\t\tabs val is %d",valToSend);
		if (param->val < 0)
			valToSend = (~valToSend + 1);
		//NSLog(@"\t\tactual val to send is %d",valToSend);
	}
	//	else the control isn't signed, i can just send it out as-is and it'll be fine
	else
		valToSend = (int)param->val;
	
	//	send the val i assembled out
	void			*bytesToSend = malloc(paramSize);
	bzero(bytesToSend,paramSize);
	memcpy(bytesToSend,&valToSend,paramSize);
	returnMe = [self _setBytes:bytesToSend sized:paramSize toControl:param->ctrlInfo];
	//NSLog(@"\t\tstraight bytes output as int: %ld",*((long *)bytesToSend));
	free(bytesToSend);
	bytesToSend = nil;
	//NSLog(@"\t\t%s - FINISHED",__func__);
	return returnMe;
}
- (void) _resetParamToDefault:(uvc_param *)param	{
	//NSLog(@"%s ... %p",__func__,param);
	/*
	if (param == &panTilt || param == &roll)	{
		NSLog(@"\t\terr: pan/tilt and roll temporarily disabled! %s",__func__);
		return;
	}
	*/
	param->val = param->def;
	[self _pushParamToDevice:param];
}


/*===================================================================================*/
#pragma mark --------------------- misc
/*------------------------------------*/


- (void) resetParamsToDefaults	{
	//NSLog(@"%s",__func__);
	[self _resetParamToDefault:&scanningMode];
	[self _resetParamToDefault:&autoExposureMode];
	[self _resetParamToDefault:&autoExposurePriority];
	[self _resetParamToDefault:&exposureTime];
	[self _resetParamToDefault:&iris];
	[self _resetParamToDefault:&autoFocus];
	[self _resetParamToDefault:&focus];
	[self _resetParamToDefault:&zoom];
	[self _resetParamToDefault:&panTilt];
	[self _resetParamToDefault:&panTiltRel];
	[self _resetParamToDefault:&roll];
	[self _resetParamToDefault:&rollRel];
	
	[self _resetParamToDefault:&backlight];
	[self _resetParamToDefault:&bright];
	[self _resetParamToDefault:&contrast];
	[self _resetParamToDefault:&gain];
	[self _resetParamToDefault:&powerLine];
	[self _resetParamToDefault:&autoHue];
	[self _resetParamToDefault:&hue];
	[self _resetParamToDefault:&saturation];
	[self _resetParamToDefault:&sharpness];
	[self _resetParamToDefault:&gamma];
	[self _resetParamToDefault:&autoWhiteBalance];
	[self _resetParamToDefault:&whiteBalance];
}
- (void) openSettingsWindow	{
	//NSLog(@"%s",__func__);
	[settingsWindow makeKeyAndOrderFront:nil];
}
- (void) closeSettingsWindow	{
	//NSLog(@"%s",__func__);
	[settingsWindow orderOut:nil];
}


/*===================================================================================*/
#pragma mark --------------------- key-val
/*------------------------------------*/


/*
- (void) setDelegate:(id<VVUVCControllerDelegate>)n	{
	if (n!=nil && [(id)n conformsToProtocol:@protocol(VVUVCControllerDelegate)])
		delegate = n;
}
- (id) delegate	{
	return delegate;
}
*/


- (void) setInterlaced:(BOOL)n	{
	//BOOL			changed = (scanningMode.val != ((n) ? 0x00 : 0x01)) ? YES : NO;
	scanningMode.val = (n) ? 0x00 : 0x01;
	[self _pushParamToDevice:&scanningMode];
	//if (changed && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (BOOL) interlaced	{
	if (!scanningMode.supported)
		return NO;
	if (scanningMode.val == 1)
		return YES;
	return NO;
}
- (BOOL) interlacedSupported	{
	return scanningMode.supported;
}
- (void) resetInterlaced	{
	[self _resetParamToDefault:&scanningMode];
}
- (void) setAutoExposureMode:(UVC_AEMode)n	{
	//long			oldVal = autoExposureMode.val;
	switch (n)	{
		case UVC_AEMode_Manual:
		case UVC_AEMode_Auto:
		case UVC_AEMode_ShutterPriority:
		case UVC_AEMode_AperturePriority:
			autoExposureMode.val = n;
			break;
		case UVC_AEMode_Undefined:
			break;
	}
	
	if (![self _pushParamToDevice:&autoExposureMode])
		[uiCtrlr _pushCameraControlStateToUI];	//	this is meant to "reload" the UI from the existing camera state if pushing a param failed (because the auto-exposure mode isn't supported).  this does not work- i think the USB device will accept the value, even though it isn't supported (the val is changing, but the behavior is simply unsupported)
	
	[self _pushParamToDevice:&exposureTime];
	//if (oldVal != autoExposureMode.val && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (UVC_AEMode) autoExposureMode	{
	if (!autoExposureMode.supported)
		return 0;
	return (UVC_AEMode)autoExposureMode.val;
}
- (BOOL) autoExposureModeSupported	{
	return autoExposureMode.supported;
}
- (void) resetAutoExposureMode	{
	[self _resetParamToDefault:&autoExposureMode];
}
- (void) setAutoExposurePriority:(BOOL)n	{
	//BOOL			changed = (autoExposurePriority.val != ((n) ? 0x01 : 0x00)) ? YES : NO;
	autoExposurePriority.val = (n) ? 0x01 : 0x00;
	[self _pushParamToDevice:&autoExposurePriority];
	//if (changed && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (BOOL) autoExposurePriority	{
	if (!autoExposurePriority.supported)
		return NO;
	if (autoExposurePriority.val == 0x01)
		return YES;
	return NO;
}
- (BOOL) autoExposurePrioritySupported	{
	return autoExposurePriority.supported;
}
- (void) resetAutoExposurePriority	{
	[self _resetParamToDefault:&autoExposurePriority];
}




- (void) setVal:(long)newVal forParam:(uvc_param *)p	{
	//long		oldVal = p->val;
	p->val = fminl(fmaxl(newVal,p->min),p->max);
	[self _pushParamToDevice:p];
	//if (oldVal!=p->val && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}




- (void) setExposureTime:(long)n	{
	[self setVal:n forParam:&exposureTime];
}
- (long) exposureTime	{
	if (!exposureTime.supported)
		return 0;
	return exposureTime.val;
}
- (BOOL) exposureTimeSupported	{
	return exposureTime.supported;
}
- (void) resetExposureTime	{
	[self _resetParamToDefault:&exposureTime];
}
- (long) minExposureTime	{
	return exposureTime.min;
}
- (long) maxExposureTime	{
	return exposureTime.max;
}
- (void) setIris:(long)n	{
	[self setVal:n forParam:&iris];
}
- (long) iris	{
	return (!iris.supported) ? 0 : iris.val;
}
- (BOOL) irisSupported	{
	return iris.supported;
}
- (void) resetIris	{
	[self _resetParamToDefault:&iris];
}
- (long) minIris	{
	return iris.min;
}
- (long) maxIris	{
	return iris.max;
}
- (void) setAutoFocus:(BOOL)n	{
	//NSLog(@"%s ... %ld",__func__,n);
	//BOOL			changed = (autoFocus.val != ((n) ? 0x01 : 0x00)) ? YES : NO;
	autoFocus.val = (n) ? 0x01 : 0x00;
	[self _pushParamToDevice:&autoFocus];
	//if (changed && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (BOOL) autoFocus	{
	if (!autoFocus.supported)
		return NO;
	if (autoFocus.val == 0x01)
		return YES;
	return NO;
}
- (BOOL) autoFocusSupported	{
	return autoFocus.supported;
}
- (void) resetAutoFocus	{
	[self _resetParamToDefault:&autoFocus];
}
- (void) setFocus:(long)n	{
	[self setVal:n forParam:&focus];
}
- (long) focus	{
	return (!focus.supported) ? 0 : focus.val;
}
- (BOOL) focusSupported	{
	return focus.supported;
}
- (void) resetFocus	{
	[self _resetParamToDefault:&focus];
}
- (long) minFocus	{
	return (!focus.supported) ? 0 : focus.min;
}
- (long) maxFocus	{
	return (!focus.supported) ? 0 : focus.max;
}
- (void) setZoom:(long)n	{
	[self setVal:n forParam:&zoom];
}
- (long) zoom	{
	return (!zoom.supported) ? 0 : zoom.val;
}
- (BOOL) zoomSupported	{
	return zoom.supported;
}
- (void) resetZoom	{
	[self _resetParamToDefault:&zoom];
}
- (long) minZoom	{
	return (!zoom.supported) ? 0 : zoom.min;
}
- (long) maxZoom	{
	return (!zoom.supported) ? 0 : zoom.max;
}
- (void) setPan:(float)n	{

}
- (long) pan	{
	return 0;
}
- (BOOL) panSupported	{
	return panTilt.supported;
}
- (void) setTilt:(long)n	{

}
- (long) tilt	{
	return 0;
}
- (BOOL) tiltSupported	{
	return panTilt.supported;
}
- (void) setRoll:(long)n	{

}
- (long) roll	{
	return 0;
}
- (BOOL) rollSupported	{
	return roll.supported;
}


- (void) setBacklight:(long)n	{
	[self setVal:n forParam:&backlight];
}
- (long) backlight	{
	return (!backlight.supported) ? 0 : backlight.val;
}
- (BOOL) backlightSupported	{
	return backlight.supported;
}
- (void) resetBacklight	{
	[self _resetParamToDefault:&backlight];
}
- (long) minBacklight	{
	return (!backlight.supported) ? 0 : backlight.min;
}
- (long) maxBacklight	{
	return (!backlight.supported) ? 0 : backlight.max;
}
- (void) setBright:(long)n	{
	[self setVal:n forParam:&bright];
}
- (long) bright	{
	return (!bright.supported) ? 0 : bright.val;
}
- (BOOL) brightSupported	{
	return bright.supported;
}
- (void) resetBright	{
	[self _resetParamToDefault:&bright];
}
- (long) minBright	{
	return (!bright.supported) ? 0 : bright.min;
}
- (long) maxBright	{
	return (!bright.supported) ? 0 : bright.max;
}
- (void) setContrast:(long)n	{
	[self setVal:n forParam:&contrast];
}
- (long) contrast	{
	return (!contrast.supported) ? 0 : contrast.val;
}
- (BOOL) contrastSupported	{
	return contrast.supported;
}
- (void) resetContrast	{
	[self _resetParamToDefault:&contrast];
}
- (long) minContrast	{
	return (!contrast.supported) ? 0 : contrast.min;
}
- (long) maxContrast	{
	return (!contrast.supported) ? 0 : contrast.max;
}
- (void) setGain:(long)n	{
	[self setVal:n forParam:&gain];
}
- (long) gain	{
	if (!gain.supported)
		return 0;
	return gain.val;
}
- (BOOL) gainSupported	{
	return gain.supported;
}
- (void) resetGain	{
	[self _resetParamToDefault:&gain];
}
- (long) minGain	{
	return (!gain.supported) ? 0 : gain.min;
}
- (long) maxGain	{
	return (!gain.supported) ? 0 : gain.max;
}
- (void) setPowerLine:(long)n	{
	[self setVal:n forParam:&powerLine];
}
- (long) powerLine	{
	if (!powerLine.supported)
		return 0;
	return powerLine.val;
}
- (BOOL) powerLineSupported	{
	return powerLine.supported;
}
- (void) resetPowerLine	{
	[self _resetParamToDefault:&powerLine];
}
- (long) minPowerLine {
	return (!powerLine.supported) ? 0 : powerLine.min;
}
- (long) maxPowerLine {
	return (!powerLine.supported) ? 0 : powerLine.max;
}
- (void) setAutoHue:(BOOL)n	{
	//BOOL			changed = (autoHue.val != ((n) ? 0x01 : 0x00)) ? YES : NO;
	autoHue.val = (n) ? 0x01 : 0x00;
	[self _pushParamToDevice:&autoHue];
	//if (changed && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (BOOL) autoHue	{
	if (!autoHue.supported)
		return NO;
	if (autoHue.val == 1)
		return YES;
	return NO;
}
- (BOOL) autoHueSupported	{
	return autoHue.supported;
}
- (void) resetAutoHue	{
	[self _resetParamToDefault:&autoHue];
}
- (void) setHue:(long)n	{
	[self setVal:n forParam:&hue];
}
- (long) hue	{
	return (!hue.supported) ? 0 : hue.val;
}
- (BOOL) hueSupported	{
	return hue.supported;
}
- (void) resetHue	{
	[self _resetParamToDefault:&hue];
}
- (long) minHue	{
	return (!hue.supported) ? 0 : hue.min;
}
- (long) maxHue	{
	return (!hue.supported) ? 0 : hue.max;
}
- (void) setSaturation:(long)n	{
	[self setVal:n forParam:&saturation];
}
- (long) saturation	{
	return (!saturation.supported) ? 0 : saturation.val;
}
- (BOOL) saturationSupported	{
	return saturation.supported;
}
- (void) resetSaturation	{
	[self _resetParamToDefault:&saturation];
}
- (long) minSaturation	{
	return (!saturation.supported) ? 0 : saturation.min;
}
- (long) maxSaturation	{
	return (!saturation.supported) ? 0 : saturation.max;
}
- (void) setSharpness:(long)n	{
	[self setVal:n forParam:&sharpness];
}
- (long) sharpness	{
	return (!sharpness.supported) ? 0 : sharpness.val;
}
- (BOOL) sharpnessSupported	{
	return sharpness.supported;
}
- (void) resetSharpness	{
	[self _resetParamToDefault:&sharpness];
}
- (long) minSharpness	{
	return (!sharpness.supported) ? 0 : sharpness.min;
}
- (long) maxSharpness	{
	return (!sharpness.supported) ? 0 : sharpness.max;
}
- (void) setGamma:(long)n	{
	[self setVal:n forParam:&gamma];
}
- (long) gamma	{
	if (!gamma.supported)
		return 0;
	return gamma.val;
}
- (BOOL) gammaSupported	{
	return gamma.supported;
}
- (void) resetGamma	{
	[self _resetParamToDefault:&gamma];
}
- (long) minGamma	{
	return (!gamma.supported) ? 0 : gamma.min;
}
- (long) maxGamma	{
	return (!gamma.supported) ? 0 : gamma.max;
}
- (void) setAutoWhiteBalance:(BOOL)n	{
	//BOOL			changed = (autoWhiteBalance.val != ((n) ? 0x01 : 0x00)) ? YES : NO;
	autoWhiteBalance.val = (n) ? 0x01 : 0x00;
	[self _pushParamToDevice:&autoWhiteBalance];
	//if (changed && delegate!=nil)
	//	[delegate VVUVCControllerParamsUpdated:self];
}
- (BOOL) autoWhiteBalance	{
	if (!autoWhiteBalance.supported)
		return NO;
	if (autoWhiteBalance.val == 1)
		return YES;
	return NO;
}
- (BOOL) autoWhiteBalanceSupported	{
	return autoWhiteBalance.supported;
}
- (void) resetAutoWhiteBalance	{
	[self _resetParamToDefault:&autoWhiteBalance];
}
- (void) setWhiteBalance:(long)n	{
	[self setVal:n forParam:&whiteBalance];
}
- (long) whiteBalance	{
	return (!whiteBalance.supported) ? 0 : whiteBalance.val;
}
- (BOOL) whiteBalanceSupported	{
	return whiteBalance.supported;
}
- (void) resetWhiteBalance	{
	[self _resetParamToDefault:&whiteBalance];
}
- (long) minWhiteBalance	{
	return whiteBalance.min;
}
- (long) maxWhiteBalance	{
	return whiteBalance.max;
}


@end

