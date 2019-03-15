#include <android/bitmap.h>
#include <android/log.h>
#include <jni.h>

#ifndef LOG_TAG
#define LOG_TAG "native"
#endif

#define LOGI(...) \
  ((void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__))
#define LOGW(...) \
  ((void)__android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__))
#define LOGE(...) \
  ((void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__))

#include "NativeFrame.hpp"

#include "frame_impl.hpp"

// BitmapUtils

// RGBA
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
} rgba_t;

void copyPixelsGray(std::uint8_t *from, rgba_t *to, uint32_t n);
void copyPixelsBGR3(std::uint8_t *from, rgba_t *to, uint32_t n);
void copyPixelsRGB3(std::uint8_t *from, rgba_t *to, uint32_t n);

void copyPixels(mynteye_jni::FrameImpl::frame_t from, rgba_t *to,
    const AndroidBitmapInfo &info) {
  if (info.width != from->width() || info.height != from->height()) {
    LOGE("Frame size is not same");
    return;
  }
  uint32_t n = info.width * info.height;
  switch (from->format()) {
    case MYNTEYE_NAMESPACE::Format::GREY:
      copyPixelsGray(from->data(), to, n);
      return;
    case MYNTEYE_NAMESPACE::Format::BGR888:
      copyPixelsBGR3(from->data(), to, n);
      return;
    case MYNTEYE_NAMESPACE::Format::RGB888:
      copyPixelsRGB3(from->data(), to, n);
      return;
    case MYNTEYE_NAMESPACE::Format::YUYV:
    default:
      LOGE("Frame format is not supported");
      return;
  }
}

CJNIEXPORT void JNICALL Java_com_slightech_mynteye_util_BitmapUtils_copyPixels(
    JNIEnv *env, jclass clazz, jobject j_frame, jobject bitmap) {
  auto frame = ::djinni_generated::NativeFrame::toCpp(env, j_frame);
  // LOGI("frame format: %dx%d", frame->Width(), frame->Height());
  auto frame_raw = std::dynamic_pointer_cast<mynteye_jni::FrameImpl>(frame)->RawFrame();

  AndroidBitmapInfo info;
  int result;
  if ((result = AndroidBitmap_getInfo(env, bitmap, &info)) < 0) {
    LOGE("AndroidBitmap_getInfo() failed, error=%d", result);
    return;
  }
  if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
    LOGE("Bitmap format is not RGBA_8888!");
    return;
  }

  void *pixels;
  if ((result = AndroidBitmap_lockPixels(env, bitmap, &pixels)) < 0) {
    LOGE("AndroidBitmap_lockPixels() failed, error=%d", result);
  }
  rgba_t *rgba_pixels = static_cast<rgba_t*>(pixels);

  copyPixels(frame_raw, rgba_pixels, info);

  AndroidBitmap_unlockPixels(env, bitmap);
}

void copyPixelsGray(std::uint8_t *from, rgba_t *to, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    std::uint8_t &gray = from[i];
    rgba_t &rgba = to[i];
    rgba.r = gray;
    rgba.g = gray;
    rgba.b = gray;
    rgba.a = 255;
  }
}

void copyPixelsBGR3(std::uint8_t *from, rgba_t *to, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    std::uint8_t *bgr = from + (i*3);
    rgba_t &rgba = to[i];
    rgba.r = *(bgr + 2);
    rgba.g = *(bgr + 1);
    rgba.b = *(bgr);
    rgba.a = 255;
  }
}

void copyPixelsRGB3(std::uint8_t *from, rgba_t *to, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    std::uint8_t *rgb = from + (i*3);
    rgba_t &rgba = to[i];
    rgba.r = *(rgb);
    rgba.g = *(rgb + 1);
    rgba.b = *(rgb + 2);
    rgba.a = 255;
  }
}
