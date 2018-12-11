// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVfoundationCamera_H
#define AVfoundationCamera_H

#import <AVFoundation/AVFoundation.h>
#import <Cocoa/Cocoa.h>
#import "VVUVCKit.h"

#include "CameraEngine.h"

static int32_t codec_table[] = { 0, 40,  0,  24, 0, 0, 0, 0, 0, 0, 'yuvs', '2vuy', 0, 'v308', '420v', '410v', 0, 0, 0, 0, 'dmb1', 'dmb1', 'mp1v', 'mp2v', 'mp4v', 'h263', 'avc1', 'dvcp', 'dvc ' };


@interface FrameGrabber : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
{
    bool new_frame;
    int cam_width, cam_height;
    int frm_width, frm_height;
    int xoff, yoff;
    unsigned char *buffer;
    bool crop;
    bool color;
}
- (id) initWithCameraSize:(int)w :(int)h :(int)b;
- (id) initWithCropSize:(int)cw :(int)ch :(int)b :(int)fw :(int)fh :(int)xo :(int)yo;
- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection;
- (unsigned char*) getFrame;

@end

class AVfoundationCamera : public CameraEngine
{

public:
    AVfoundationCamera(CameraConfig* cam_cfg);
    ~AVfoundationCamera();
	
	static int getDeviceCount();
	static std::vector<CameraConfig> getCameraConfigs(int dev_id = -1);
	static CameraEngine* getCamera(CameraConfig* cam_cfg);
	
	bool initCamera();
	bool startCamera();
	unsigned char* getFrame();
	bool stopCamera();
	bool stillRunning();
	bool resetCamera();
	bool closeCamera();

    int getCameraSettingStep(int mode);
    bool setCameraSettingAuto(int mode, bool flag);
    bool getCameraSettingAuto(int mode);
    bool setCameraSetting(int mode, int value);
    int getCameraSetting(int mode);
    int getMaxCameraSetting(int mode);
    int getMinCameraSetting(int mode);
    bool setDefaultCameraSetting(int mode);
    int getDefaultCameraSetting(int mode);
    bool hasCameraSetting(int mode);
    bool hasCameraSettingAuto(int mode);
    
	bool showSettingsDialog(bool lock);
    
private:
    
    bool disconnected;
    
    VVUVCController				*uvcController;
    FrameGrabber                *grabber;
    
    AVCaptureSession			*session;
    AVCaptureDeviceInput		*videoDeviceInput;
    AVCaptureVideoDataOutput    *videoOutput;
    AVCaptureDevice             *videoDevice;
};
#endif
