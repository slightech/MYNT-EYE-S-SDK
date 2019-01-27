package com.slightech.mynteye.demo.camera;

import android.os.Handler;
import android.os.HandlerThread;
import com.slightech.mynteye.Device;
import com.slightech.mynteye.DeviceUsbInfo;
import com.slightech.mynteye.MotionData;
import com.slightech.mynteye.Source;
import com.slightech.mynteye.Stream;
import com.slightech.mynteye.StreamData;
import com.slightech.mynteye.StreamRequest;
import java.util.ArrayList;

public final class Mynteye implements Runnable {

  private Device mDevice;

  private HandlerThread mBackgroundThread;
  private Handler mBackgroundHandler;

  private boolean mOpened;

  public interface OnStreamDataReceiveListener {
    void onStreamDataReceive(Stream stream, StreamData data, Handler handler);
    void onStreamLeftReceive(StreamData data, Handler handler);
    void onStreamRightReceive(StreamData data, Handler handler);
  }

  public interface OnMotionDataReceiveListener {
    void onMotionDataReceive(ArrayList<MotionData> datas, Handler handler);
  }

  private OnStreamDataReceiveListener mOnStreamDataReceiveListener;
  private OnMotionDataReceiveListener mOnMotionDataReceiveListener;

  public Mynteye(DeviceUsbInfo info) {
    mDevice = Device.create(info);
    mOpened = false;
  }

  public void setOnStreamDataReceiveListener(OnStreamDataReceiveListener l) {
    mOnStreamDataReceiveListener = l;
  }

  public void setOnMotionDataReceiveListener(OnMotionDataReceiveListener l) {
    mOnMotionDataReceiveListener = l;
  }

  public ArrayList<StreamRequest> getStreamRequests() {
    return mDevice.getStreamRequests();
  }

  public void open(StreamRequest request) {
    if (mOpened) return;
    mOpened = true;
    startBackgroundThread();

    mDevice.configStreamRequest(request);
    mDevice.enableMotionDatas(Integer.MAX_VALUE);
    mDevice.start(Source.ALL);

    mBackgroundHandler.post(this);
  }

  public void close() {
    if (!mOpened) return;
    mOpened = false;
    stopBackgroundThread();
    mDevice.stop(Source.ALL);
  }

  @Override
  public void run() {
    //Timber.i("wait streams");
    mDevice.waitForStreams();

    //Timber.i("get streams");
    {
      StreamData data = mDevice.getStreamData(Stream.LEFT);
      if (mOnStreamDataReceiveListener != null) {
        mOnStreamDataReceiveListener.onStreamDataReceive(Stream.LEFT, data, mBackgroundHandler);
        mOnStreamDataReceiveListener.onStreamLeftReceive(data, mBackgroundHandler);
      }
    }
    {
      StreamData data = mDevice.getStreamData(Stream.RIGHT);
      if (mOnStreamDataReceiveListener != null) {
        mOnStreamDataReceiveListener.onStreamDataReceive(Stream.RIGHT, data, mBackgroundHandler);
        mOnStreamDataReceiveListener.onStreamRightReceive(data, mBackgroundHandler);
      }
    }

    //Timber.i("get motions");
    {
      ArrayList<MotionData> datas = mDevice.getMotionDatas();
      if (mOnMotionDataReceiveListener != null) {
        mOnMotionDataReceiveListener.onMotionDataReceive(datas, mBackgroundHandler);
      }
    }

    if (mOpened) mBackgroundHandler.post(this);
  }

  private void startBackgroundThread() {
    mBackgroundThread = new HandlerThread("MynteyeBackground");
    mBackgroundThread.start();
    mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
  }

  private void stopBackgroundThread() {
    mBackgroundThread.quitSafely();
    //mBackgroundThread.interrupt();
    try {
      mBackgroundHandler.removeCallbacksAndMessages(null);
      mBackgroundThread.join();
      mBackgroundThread = null;
      mBackgroundHandler = null;
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

}
