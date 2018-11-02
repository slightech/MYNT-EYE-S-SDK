#import <Cocoa/Cocoa.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>




extern CVOpenGLTextureCacheRef		_textureCache;




@protocol AVCaptureVideoSourceDelegate
- (void) listOfStaticSourcesUpdated:(id)videoSource;
@end




@interface AVCaptureVideoSource : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>	{
	OSSpinLock							propLock;
	id <AVCaptureVideoSourceDelegate>	propDelegate;
	BOOL								propRunning;
	
	AVCaptureDeviceInput				*propDeviceInput;
	AVCaptureSession					*propSession;
	AVCaptureVideoDataOutput			*propOutput;
	dispatch_queue_t					propQueue;
	CVOpenGLTextureRef					propTexture;
}

- (void) loadDeviceWithUniqueID:(NSString *)n;
- (void) stop;
- (void) _stop;

- (BOOL) running;
- (void) setDelegate:(id<AVCaptureVideoSourceDelegate>)n;
- (NSArray *) arrayOfSourceMenuItems;

- (CVOpenGLTextureRef) safelyGetRetainedTextureRef;

@end
