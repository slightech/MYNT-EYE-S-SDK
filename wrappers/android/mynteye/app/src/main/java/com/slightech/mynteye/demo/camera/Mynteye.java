package com.slightech.mynteye.demo.camera;

import android.os.Handler;
import android.os.HandlerThread;
import com.slightech.mynteye.Device;
import com.slightech.mynteye.DeviceUsbInfo;
import com.slightech.mynteye.Info;
import com.slightech.mynteye.MotionData;
import com.slightech.mynteye.MotionIntrinsics;
import com.slightech.mynteye.Option;
import com.slightech.mynteye.Source;
import com.slightech.mynteye.Stream;
import com.slightech.mynteye.StreamData;
import com.slightech.mynteye.StreamRequest;
import java.util.ArrayList;
import java.util.Map;
import timber.log.Timber;

public final class Mynteye implements Runnable {

  private Device mDevice;

  private HandlerThread mBackgroundThread;
  private Handler mBackgroundHandler;

  private boolean mOpened;
  private boolean mImuEnabled;

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

  private StreamRequest mStreamRequest;

  public Mynteye(DeviceUsbInfo info) {
    mDevice = Device.create(info);
    mOpened = false;
    mImuEnabled = false;
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

  public String getDeviceInfos() {
    StringBuffer sb = new StringBuffer();
    for (Info info : Info.values()) {
      sb.append(info.toString());
      sb.append(": ");
      sb.append(mDevice.getInfo(info));
      sb.append('\n');
    }
    return sb.toString();
  }

  public String getImageParams() {
    StringBuffer sb = new StringBuffer();
    sb.append(Stream.LEFT).append('\n').append(mDevice.getIntrinsics(Stream.LEFT));
    sb.append("\n\n");
    sb.append(Stream.RIGHT).append('\n').append(mDevice.getIntrinsics(Stream.RIGHT));
    sb.append("\n\n");
    sb.append(Stream.LEFT).append(" > ").append(Stream.RIGHT);
    sb.append('\n');
    sb.append(mDevice.getExtrinsics(Stream.LEFT, Stream.RIGHT));
    return sb.toString();
  }

  public String getImuParams() {
    StringBuffer sb = new StringBuffer();
    MotionIntrinsics in = mDevice.getMotionIntrinsics();
    sb.append("Accel\n").append(in.getAccel());
    sb.append("\n\n");
    sb.append("Gyro\n").append(in.getGyro());
    sb.append("\n\n");
    sb.append("Imu > ").append(Stream.LEFT).append('\n')
        .append(mDevice.getMotionExtrinsics(Stream.LEFT));
    return sb.toString();
  }

  public String getOptionInfos() {
    StringBuffer sb = new StringBuffer();
    for (Option op : Option.values()) {
      if (!mDevice.supportsOption(op)) {
        continue;
      }
      sb.append(op.toString());
      sb.append(": ");
      sb.append(mDevice.getOptionValue(op));
      sb.append("\n  ");
      sb.append(mDevice.getOptionInfo(op));
      sb.append('\n');
    }
    return sb.toString();
  }

  public boolean isOpened() {
    return mOpened;
  }

  public boolean isImuEnabled() {
    return mImuEnabled;
  }

  public void setImuEnabled(boolean enabled) {
    mImuEnabled = enabled;
    if (mOpened) {
      Timber.w("Will enable imu when open next time");
    }
  }

  public void open() {
    if (mOpened) return;
    if (mStreamRequest == null) {
      Timber.w("Should open with stream request");
      return;
    }
    open(mStreamRequest);
  }

  public void open(StreamRequest request) {
    if (mOpened) return;
    mOpened = true;
    mStreamRequest = request;

    startBackgroundThread();

    mDevice.configStreamRequest(request);
    if (mImuEnabled) {
      mDevice.enableMotionDatas(Integer.MAX_VALUE);
      mDevice.start(Source.ALL);
    } else {
      mDevice.start(Source.VIDEO_STREAMING);
    }

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
    if (mImuEnabled) {
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
