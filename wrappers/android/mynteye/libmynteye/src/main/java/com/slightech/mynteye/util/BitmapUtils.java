package com.slightech.mynteye.util;

import android.graphics.Bitmap;
import com.slightech.mynteye.Frame;

public class BitmapUtils {

  public static native void copyPixels(Frame from, Bitmap to);

}
