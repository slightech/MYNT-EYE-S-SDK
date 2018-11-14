#import <Cocoa/Cocoa.h>
#import "VVUVCUIElement.h"




@interface VVUVCUIController : NSObject <VVUVCUIElementDelegate> {
	IBOutlet id				device;
	
	IBOutlet NSPopUpButton	*autoExpButton;
	IBOutlet NSButton		*expPriorityButton;
	IBOutlet NSButton		*autoFocusButton;
	//IBOutlet NSSlider		*panSlider;
	//IBOutlet NSSlider		*tiltSlider;
	//IBOutlet NSSlider		*rollSlider;
	
	IBOutlet NSButton		*autoHueButton;
	IBOutlet NSButton		*autoWBButton;
	
	IBOutlet VVUVCUIElement		*expElement;
	IBOutlet VVUVCUIElement		*irisElement;
	IBOutlet VVUVCUIElement		*focusElement;
	IBOutlet VVUVCUIElement		*zoomElement;

	IBOutlet VVUVCUIElement		*backlightElement;
	IBOutlet VVUVCUIElement		*brightElement;
	IBOutlet VVUVCUIElement		*contrastElement;
	IBOutlet VVUVCUIElement		*powerElement;
	IBOutlet VVUVCUIElement		*gammaElement;
	IBOutlet VVUVCUIElement		*hueElement;
	IBOutlet VVUVCUIElement		*satElement;
	IBOutlet VVUVCUIElement		*sharpElement;
	IBOutlet VVUVCUIElement		*gainElement;
	IBOutlet VVUVCUIElement		*wbElement;
}

- (IBAction) buttonUsed:(id)sender;
- (IBAction) popUpButtonUsed:(id)sender;

- (IBAction) resetToDefaults:(id)sender;

- (void) _pushCameraControlStateToUI;

@end
