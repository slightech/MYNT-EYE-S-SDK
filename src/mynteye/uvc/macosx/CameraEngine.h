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

#ifndef SRC_MYNTEYE_UVC_MACOSX_CAMERAENGINE_H_
#define SRC_MYNTEYE_UVC_MACOSX_CAMERAENGINE_H_

#include <limits.h>
#include <math.h>
#include <CoreFoundation/CFBundle.h>

#include <list>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>

#define SAT(c) \
if (c & (~255)) { \
  if (c < 0) { \
    c = 0; \
  } else { \
    c = 255; \
  } \
}
#define HBT(x) (unsigned char)((x)>>8)

#define KEY_A 4
#define KEY_B 5
#define KEY_C 6
#define KEY_D 7
#define KEY_E 8
#define KEY_F 9
#define KEY_G 10
#define KEY_H 11
#define KEY_I 12
#define KEY_J 13
#define KEY_K 14
#define KEY_L 15
#define KEY_M 16
#define KEY_N 17
#define KEY_O 18
#define KEY_P 19
#define KEY_Q 20
#define KEY_R 21
#define KEY_S 22
#define KEY_T 23
#define KEY_U 24
#define KEY_V 25
#define KEY_W 26
#define KEY_X 27
#define KEY_Y 29
#define KEY_Z 28

#define KEY_SPACE 44
#define KEY_RIGHT 79
#define KEY_LEFT 80
#define KEY_DOWN 81
#define KEY_UP 82

#define WIDTH 640
#define HEIGHT 480

#define SETTING_DEFAULT -100000
#define SETTING_AUTO    -200000
#define SETTING_MIN     -300000
#define SETTING_MAX     -400000
#define SETTING_OFF     -500000

#define FORMAT_UNSUPPORTED  -1
#define FORMAT_UNKNOWN  0
#define FORMAT_GRAY     1
#define FORMAT_GRAY16   2
#define FORMAT_RGB      3
#define FORMAT_RGB16    4
#define FORMAT_GRAY16S  5
#define FORMAT_RGB16S   6
#define FORMAT_RAW8     7
#define FORMAT_RAW16    8
#define FORMAT_RGBA     9
#define FORMAT_YUYV    10
#define FORMAT_UYVY    11
#define FORMAT_YUV411  12
#define FORMAT_YUV444  13
#define FORMAT_420P    14
#define FORMAT_410P    15
#define FORMAT_YVYU    16
#define FORMAT_YUV211  17
#define FORMAT_JPEG    20
#define FORMAT_MJPEG   21
#define FORMAT_MPEG    22
#define FORMAT_MPEG2   23
#define FORMAT_MPEG4   24
#define FORMAT_H263    25
#define FORMAT_H264    26
#define FORMAT_DVPAL   30
#define FORMAT_DVNTSC  31
#define FORMAT_MAX     31

extern const char* fstr[];
extern const char* dstr[];

#define DRIVER_DEFAULT  0
#define DRIVER_DC1394   1
#define DRIVER_PS3EYE   2
#define DRIVER_RASPI    3
#define DRIVER_UVCCAM   4
#define DRIVER_FILE    10
#define DRIVER_FOLDER  11

#define VALUE_INCREASE   79
#define VALUE_DECREASE   80
#define SETTING_NEXT     81
#define SETTING_PREVIOUS 82

enum CameraSetting {
    BRIGHTNESS,
    CONTRAST,
    SHARPNESS,
    AUTO_GAIN,
    GAIN,
    AUTO_EXPOSURE,
    EXPOSURE,
    SHUTTER,
    AUTO_FOCUS,
    FOCUS,
    AUTO_WHITE,
    WHITE,
    GAMMA,
    POWERLINE,
    BACKLIGHT,
    SATURATION,
    AUTO_HUE,
    COLOR_HUE,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE };
#define MODE_MIN BRIGHTNESS
#define MODE_MAX COLOR_BLUE

struct CameraConfig {
  char path[256];

  int driver;
  int device;

  char name[256];
  char src[256];

  bool color;
  bool frame;

  int cam_format;
  int src_format;
  int buf_format;

  int cam_width;
  int cam_height;
  float cam_fps;

  int frame_width;
  int frame_height;
  int frame_xoff;
  int frame_yoff;
  int frame_mode;

  int brightness;
  int contrast;
  int sharpness;

  int gain;
  int shutter;
  int exposure;
  int focus;
  int gamma;
  int white;
  int powerline;
  int backlight;

  int saturation;
  int hue;
  int red;
  int blue;
  int green;
  bool force;

  bool operator < (const CameraConfig& c) const {
      // if (device < c.device) return true;
      // if (cam_format < c.cam_format) return true;

    if (cam_width > c.cam_width ||
        (cam_width == c.cam_width && cam_height < c.cam_height))
      return true;

    if (cam_width == c.cam_width && cam_height == c.cam_height) {
      return (cam_fps > c.cam_fps);
    } else {return false;}
  }
};

class CameraEngine {
 public:
  explicit CameraEngine(CameraConfig *cam_cfg) {
    cfg = cam_cfg;
    settingsDialog = false;

    if (cfg->color) {
      cfg->buf_format = FORMAT_RGB;
    } else {
      cfg->buf_format = FORMAT_GRAY;
    }
  }

  virtual ~CameraEngine() {}

  virtual bool initCamera() = 0;
  virtual bool startCamera() = 0;
  virtual unsigned char* getFrame() = 0;
  virtual bool stopCamera() = 0;
  virtual bool resetCamera() = 0;
  virtual bool closeCamera() = 0;
  virtual bool stillRunning() = 0;

  void printInfo();
  static void setMinMaxConfig(CameraConfig *cam_cfg,
      std::vector<CameraConfig> cfg_list);

  virtual int getCameraSettingStep(int mode) = 0;
  virtual int getMinCameraSetting(int mode) = 0;
  virtual int getMaxCameraSetting(int mode) = 0;
  virtual int getCameraSetting(int mode) = 0;
  virtual bool setCameraSetting(int mode, int value) = 0;
  virtual bool setCameraSettingAuto(int mode, bool flag) = 0;
  virtual bool getCameraSettingAuto(int mode) = 0;
  virtual bool setDefaultCameraSetting(int mode) = 0;
  virtual int getDefaultCameraSetting(int mode) = 0;
  virtual bool hasCameraSetting(int mode) = 0;
  virtual bool hasCameraSettingAuto(int mode) = 0;

  virtual bool showSettingsDialog(bool lock);
  virtual void control(unsigned char key);

  inline int getId() { return cfg->device; }
  inline int getFps() { return static_cast<int>(floor(cfg->cam_fps+0.5f)); }
  inline int getWidth() { return cfg->frame_width; }
  inline int getHeight() { return cfg->frame_height; }
  inline int getFormat() { return cfg->buf_format; }
  inline char* getName() { return cfg->name; }

 protected:
  CameraConfig *cfg;

  unsigned char* frm_buffer;
  unsigned char* cam_buffer;

  int lost_frames, timeout;

  bool running;
  bool settingsDialog;
  int currentCameraSetting;

  void crop(int width, int height, unsigned char *src,
      unsigned char *dest, int bytes);
  void flip(int width, int height, unsigned char *src,
      unsigned char *dest, int bytes);
  void flip_crop(int width, int height, unsigned char *src,
      unsigned char *dest, int bytes);

  void rgb2gray(int width, int height, unsigned char *src,
      unsigned char *dest);
  void crop_rgb2gray(int width, unsigned char *src, unsigned char *dest);
  void flip_rgb2gray(int width, int height, unsigned char *src,
      unsigned char *dest);
  void flip_crop_rgb2gray(int width, unsigned char *src,
      unsigned char *dest);

  void uyvy2gray(int width, int height, unsigned char *src,
      unsigned char *dest);
  void crop_uyvy2gray(int width, unsigned char *src, unsigned char *dest);
  void yuyv2gray(int width, int height, unsigned char *src,
      unsigned char *dest);
  void crop_yuyv2gray(int width, unsigned char *src, unsigned char *dest);
  void yuv2gray(int width, int height, unsigned char *src, unsigned char *dest);
  void crop_yuv2gray(int width, unsigned char *src, unsigned char *dest);

  void gray2rgb(int width, int height, unsigned char *src, unsigned char *dest);
  void crop_gray2rgb(int width, unsigned char *src, unsigned char *dest);
  void uyvy2rgb(int width, int height, unsigned char *src, unsigned char *dest);
  void crop_uyvy2rgb(int width, unsigned char *src, unsigned char *dest);
  void yuyv2rgb(int width, int height, unsigned char *src, unsigned char *dest);
  void crop_yuyv2rgb(int width, unsigned char *src, unsigned char *dest);

  void grayw2rgb(int width, int height, unsigned char *src,
      unsigned char *dest);
  void crop_grayw2rgb(int width, unsigned char *src, unsigned char *dest);
  void grayw2gray(int width, int height, unsigned char *src,
      unsigned char *dest);
  void crop_grayw2gray(int width, unsigned char *src, unsigned char *dest);

  void resetCameraSettings();
  void applyCameraSettings();
  void applyCameraSetting(int mode, int value);
  void updateSettings();
  int updateSetting(int mode);

  void setupFrame();

  int default_brightness;
  int default_contrast;
  int default_sharpness;

  int default_gain;
  int default_shutter;
  int default_exposure;
  int default_focus;
  int default_gamma;
  int default_powerline;
  int default_white;
  int default_backlight;

  int default_saturation;
  int default_hue;
  int default_red;
  int default_blue;
  int default_green;

  int ctrl_min;
  int ctrl_max;
  int ctrl_val;
};
#endif  // SRC_MYNTEYE_UVC_MACOSX_CAMERAENGINE_H_
