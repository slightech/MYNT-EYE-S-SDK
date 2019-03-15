package com.slightech.mynteye.demo;

import android.app.Application;
import timber.log.Timber;

//import com.stericson.RootShell.RootShell;

public class MyApplication extends Application {

  static {
    try {
      System.loadLibrary("mynteye_jni");
    } catch (UnsatisfiedLinkError e) {
      System.err.println("mynteye_jni library failed to load.\n" + e);
    }
  }

  @Override public void onCreate() {
    super.onCreate();
    Timber.plant(new Timber.DebugTree());
    //RootShell.debugMode = true;
  }

  @Override public void onLowMemory() {
    super.onLowMemory();
  }

  @Override public void onTrimMemory(int level) {
    super.onTrimMemory(level);
  }

  @Override public void onTerminate() {
    super.onTerminate();
  }
}
