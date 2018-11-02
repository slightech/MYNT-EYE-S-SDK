#import "VVUVCUIController.h"
#import "VVUVCController.h"




@implementation VVUVCUIController


- (id) init	{
	//NSLog(@"%s",__func__);
	if (self = [super init])	{
		return self;
	}
	[self release];
	return nil;
}
- (void) awakeFromNib	{
	[expElement setTitle:@"Exposure Time"];
	[irisElement setTitle:@"Iris"];
	[focusElement setTitle:@"Focus"];
	[zoomElement setTitle:@"Zoom"];

	[backlightElement setTitle:@"Backlight Compensation"];
	[brightElement setTitle:@"Brightness"];
	[contrastElement setTitle:@"Contrast"];
	[gainElement setTitle:@"Gain"];
	[powerElement setTitle:@"Power Line Frequency"];
	[hueElement setTitle:@"Hue"];
	[satElement setTitle:@"Saturation"];
	[sharpElement setTitle:@"Sharpness"];
	[gammaElement setTitle:@"Gamma"];
	[wbElement setTitle:@"White Balance"];
}
- (void) dealloc	{
	[super dealloc];
}


- (void) controlElementChanged:(id)sender	{
	//NSLog(@"%s",__func__);
	if (sender == expElement)	{
		[device setExposureTime:[sender val]];
	}
	else if (sender == irisElement)	{
		[device setIris:[sender val]];
	}
	else if (sender == focusElement)	{
		[device setFocus:[sender val]];
	}
	else if (sender == zoomElement)	{
		[device setZoom:[sender val]];
	}
	else if (sender == backlightElement)	{
		[device setBacklight:[sender val]];
	}
	else if (sender == brightElement)	{
		[device setBright:[sender val]];
	}
	else if (sender == contrastElement)	{
		[device setContrast:[sender val]];
	}
	else if (sender == gainElement)	{
		[device setGain:[sender val]];
	}
	else if (sender == powerElement)	{
		[device setPowerLine:[sender val]];
	}
	else if (sender == hueElement)	{
		[device setHue:[sender val]];
	}
	else if (sender == satElement)	{
		[device setSaturation:[sender val]];
	}
	else if (sender == sharpElement)	{
		[device setSharpness:[sender val]];
	}
	else if (sender == gammaElement)	{
		[device setGamma:[sender val]];
	}
	else if (sender == wbElement)	{
		[device setWhiteBalance:[sender val]];
	}
	[self _pushCameraControlStateToUI];
}
- (IBAction) buttonUsed:(id)sender	{
	if (sender == expPriorityButton)	{
		[device setAutoExposurePriority:([sender intValue]==NSOnState) ? YES : NO];
	}
	else if (sender == autoFocusButton)	{
		if ([sender intValue] == NSOnState)	{
			[device setAutoFocus:YES];
		}
		else	{
			[device setAutoFocus:NO];
		}
		[self _pushCameraControlStateToUI];
	}
	else if (sender == autoHueButton)	{
		if ([sender intValue] == NSOnState)	{
			[device setAutoHue:YES];
		}
		else	{
			[device setAutoHue:NO];
		}
		[self _pushCameraControlStateToUI];
	}
	else if (sender == autoWBButton)	{
		if ([sender intValue] == NSOnState)	{
			[device setAutoWhiteBalance:YES];
		}
		else	{
			[device setAutoWhiteBalance:NO];
		}
		[self _pushCameraControlStateToUI];
	}
}
- (IBAction) popUpButtonUsed:(id)sender	{
	//NSLog(@"%s ... %d",__func__,[sender indexOfSelectedItem]);
	if (sender == autoExpButton)	{
		int		selectedIndex = (int)[sender indexOfSelectedItem];
		if (selectedIndex == 0)	{
			[device setAutoExposureMode:UVC_AEMode_Manual];
		}
		else if (selectedIndex == 1)	{
			[device setAutoExposureMode:UVC_AEMode_Auto];
		}
		else if (selectedIndex == 2)	{
			[device setAutoExposureMode:UVC_AEMode_ShutterPriority];
		}
		else if (selectedIndex == 3)	{
			[device setAutoExposureMode:UVC_AEMode_AperturePriority];
		}
		[self _pushCameraControlStateToUI];
	}
}


- (IBAction) resetToDefaults:(id)sender	{
	//NSLog(@"%s",__func__);
	[device resetParamsToDefaults];
	[self _pushCameraControlStateToUI];
}


- (void) _pushCameraControlStateToUI	{
	//NSLog(@"%s",__func__);
	

	if ([device exposureTimeSupported])	{
		[expElement setMin:(int)[device minExposureTime]];
		[expElement setMax:(int)[device maxExposureTime]];
		[expElement setVal:(int)[device exposureTime]];
	}
	[expElement setEnabled:[device exposureTimeSupported]];

	if ([device irisSupported])	{
		[irisElement setMin:(int)[device minIris]];
		[irisElement setMax:(int)[device maxIris]];
		[irisElement setVal:(int)[device iris]];
	}
	[irisElement setEnabled:[device irisSupported]];

	if ([device zoomSupported])	{
		[zoomElement setMin:(int)[device minZoom]];
		[zoomElement setMax:(int)[device maxZoom]];
		[zoomElement setVal:(int)[device zoom]];
	}
	[zoomElement setEnabled:[device zoomSupported]];

	if ([device backlightSupported])	{
		[backlightElement setMin:(int)[device minBacklight]];
		[backlightElement setMax:(int)[device maxBacklight]];
		[backlightElement setVal:(int)[device backlight]];
	}
	[backlightElement setEnabled:[device backlightSupported]];

	if ([device brightSupported])	{
		[brightElement setMin:(int)[device minBright]];
		[brightElement setMax:(int)[device maxBright]];
		[brightElement setVal:(int)[device bright]];
	}
	[brightElement setEnabled:[device brightSupported]];

	if ([device contrastSupported])	{
		[contrastElement setMin:(int)[device minContrast]];
		[contrastElement setMax:(int)[device maxContrast]];
		[contrastElement setVal:(int)[device contrast]];
	}
	[contrastElement setEnabled:[device contrastSupported]];
	
	if ([device gainSupported])	{
		[gainElement setMin:(int)[device minGain]];
		[gainElement setMax:(int)[device maxGain]];
		[gainElement setVal:(int)[device gain]];
	}
	[gainElement setEnabled:[device gainSupported]];

	if ([device powerLineSupported])	{
		[powerElement setMin:(int)[device minPowerLine]];
		[powerElement setMax:(int)[device maxPowerLine]];
		[powerElement setVal:(int)[device powerLine]];
	}
	[powerElement setEnabled:[device powerLineSupported]];
	
	if ([device saturationSupported])	{
		[satElement setMin:(int)[device minSaturation]];
		[satElement setMax:(int)[device maxSaturation]];
		[satElement setVal:(int)[device saturation]];
	}
	[satElement setEnabled:[device saturationSupported]];

	if ([device sharpnessSupported])	{
		[sharpElement setMin:(int)[device minSharpness]];
		[sharpElement setMax:(int)[device maxSharpness]];
		[sharpElement setVal:(int)[device sharpness]];
	}
	[sharpElement setEnabled:[device sharpnessSupported]];

	if ([device gammaSupported])	{
		[gammaElement setMin:(int)[device minGamma]];
		[gammaElement setMax:(int)[device maxGamma]];
		[gammaElement setVal:(int)[device gamma]];
	}
	
	[expPriorityButton setEnabled:[device autoExposurePrioritySupported]];
	[expPriorityButton setIntValue:([device autoExposurePriority]) ? NSOnState : NSOffState];
	
	[autoFocusButton setEnabled:([device autoFocusSupported]) ? YES : NO];
	[autoFocusButton setIntValue:([device autoFocus]) ? NSOnState : NSOffState];
	
	
	BOOL			enableFocusElement = NO;
	if ([device autoFocusSupported])	{
		[autoFocusButton setEnabled:YES];
		if ([device autoFocus])	{
			[autoFocusButton setIntValue:NSOnState];
		}
		else	{
			[autoFocusButton setIntValue:NSOffState];
			if ([device focusSupported])
				enableFocusElement = YES;
		}
	}
	else	{
		[autoFocusButton setEnabled:NO];
		[autoFocusButton setIntValue:NSOffState];
		if ([device focusSupported])
			enableFocusElement = YES;
	}
	[focusElement setEnabled:enableFocusElement];
	if (enableFocusElement)	{
		[focusElement setMin:(int)[device minFocus]];
		[focusElement setMax:(int)[device maxFocus]];
		[focusElement setVal:(int)[device focus]];
	} else [focusElement setVal:0];

	
	BOOL			enableHueElement = NO;
	if ([device autoHueSupported])	{
		[autoHueButton setEnabled:YES];
		if ([device autoHue])	{
			[autoHueButton setIntValue:NSOnState];
		}
		else	{
			[autoHueButton setIntValue:NSOffState];
			if ([device hueSupported])
				enableHueElement = YES;
		}
	}
	else	{
		[autoHueButton setEnabled:NO];
		[autoHueButton setIntValue:NSOffState];
		if ([device hueSupported])
			enableHueElement = YES;
	}
	[hueElement setEnabled:enableHueElement];
	if (enableHueElement)	{
		[hueElement setMin:(int)[device minHue]];
		[hueElement setMax:(int)[device maxHue]];
		[hueElement setVal:(int)[device hue]];
	} else [hueElement setVal:0];


	BOOL			enableWBElement = NO;
	if ([device autoWhiteBalanceSupported])	{
		[autoWBButton setEnabled:YES];
		if ([device autoWhiteBalance])	{
			[autoWBButton setIntValue:NSOnState];
		}
		else	{
			[autoWBButton setIntValue:NSOffState];
			if ([device whiteBalanceSupported])
				enableWBElement = YES;
		}
	}
	else	{
		[autoWBButton setEnabled:NO];
		[autoWBButton setIntValue:NSOffState];
		if ([device whiteBalanceSupported])
			enableWBElement = YES;
	}
	[wbElement setEnabled:enableWBElement];
	if (enableWBElement)	{
		[wbElement setMin:(int)[device minWhiteBalance]];
		[wbElement setMax:(int)[device maxWhiteBalance]];
		[wbElement setVal:(int)[device whiteBalance]];
	} else [wbElement setVal:0];


	UVC_AEMode		aeMode = [device autoExposureMode];
	switch (aeMode)	{
		case UVC_AEMode_Undefined:	//	hide both
			[autoExpButton selectItemAtIndex:0];
			[expElement setEnabled:NO];
			[irisElement setEnabled:NO];
			break;
		case UVC_AEMode_Manual:	//	show both
			[autoExpButton selectItemAtIndex:0];
			[expElement setEnabled:(YES && [device exposureTimeSupported])];
			[irisElement setEnabled:(YES && [device irisSupported])];
			break;
		case UVC_AEMode_Auto:	//	hide both
			[autoExpButton selectItemAtIndex:1];
			[expElement setEnabled:NO];
			[irisElement setEnabled:NO];
			[expElement setVal:0];
			break;
		case UVC_AEMode_ShutterPriority:
			[autoExpButton selectItemAtIndex:2];
			[expElement setEnabled:(YES && [device exposureTimeSupported])];
			[irisElement setEnabled:NO];
			break;
		case UVC_AEMode_AperturePriority:
			[autoExpButton selectItemAtIndex:3];
			[expElement setEnabled:NO];
			[irisElement setEnabled:(YES && [device irisSupported])];
			[expElement setVal:0];
			break;
	}
}


@end
