package com.slightech.mynteye.demo.ui;

import android.graphics.Bitmap;
import android.hardware.usb.UsbDevice;
import android.os.Bundle;
import android.os.Handler;
import android.text.TextUtils;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AlertDialog;
import butterknife.BindView;
import butterknife.ButterKnife;
import com.slightech.mynteye.DeviceUsbInfo;
import com.slightech.mynteye.Frame;
import com.slightech.mynteye.ImuData;
import com.slightech.mynteye.MotionData;
import com.slightech.mynteye.Stream;
import com.slightech.mynteye.StreamData;
import com.slightech.mynteye.StreamRequest;
import com.slightech.mynteye.demo.R;
import com.slightech.mynteye.demo.camera.Mynteye;
import com.slightech.mynteye.usb.CameraDialog;
import com.slightech.mynteye.usb.USBMonitor;
import com.slightech.mynteye.usb.USBMonitor.OnDeviceConnectListener;
import com.slightech.mynteye.usb.USBMonitor.UsbControlBlock;
import com.slightech.mynteye.util.BitmapUtils;
import java.util.ArrayList;
import java.util.Locale;
import timber.log.Timber;

public class MainActivity extends BaseActivity implements CameraDialog.CameraDialogParent,
    Mynteye.OnStreamDataReceiveListener, Mynteye.OnMotionDataReceiveListener {

  @BindView(R.id.text) TextView mTextView;
  @BindView(R.id.image_left) ImageView mLeftImageView;
  @BindView(R.id.image_right) ImageView mRightImageView;

  private USBMonitor mUSBMonitor;

  private Mynteye mMynteye;
  private Bitmap mLeftBitmap, mRightBitmap;

  private boolean mImuEnabled;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    ButterKnife.bind(this);
    mUSBMonitor = new USBMonitor(this, mOnDeviceConnectListener);
  }

  @Override
  protected void onStart() {
    super.onStart();
    mUSBMonitor.register();
    if (mMynteye == null) {
      //actionOpen();
    }
  }

  @Override
  protected void onStop() {
    super.onStop();
    if (mUSBMonitor != null) {
      mUSBMonitor.unregister();
    }
  }

  @Override
  protected void onDestroy() {
    if (mMynteye != null) {
      mMynteye.close();
      mMynteye = null;
    }
    if (mUSBMonitor != null) {
      mUSBMonitor.destroy();
      mUSBMonitor = null;
    }
    super.onDestroy();
  }

  private final OnDeviceConnectListener mOnDeviceConnectListener = new OnDeviceConnectListener() {

    @Override
    public void onAttach(final UsbDevice device) {
      toast("USB_DEVICE_ATTACHED");
    }

    @Override
    public void onConnect(final UsbDevice device, final UsbControlBlock ctrlBlock, final boolean createNew) {
      toast(String.format(Locale.getDefault(), "CONNECT, %s: %s", ctrlBlock.getProductName(), ctrlBlock.getSerial()));
      openDevice(new DeviceUsbInfo(
          ctrlBlock.getVenderId(),
          ctrlBlock.getProductId(),
          ctrlBlock.getFileDescriptor(),
          ctrlBlock.getBusNum(),
          ctrlBlock.getDevNum(),
          getUSBFSName(ctrlBlock),
          ctrlBlock.getProductName(),
          ctrlBlock.getSerial()));
    }

    @Override
    public void onDisconnect(final UsbDevice device, final UsbControlBlock ctrlBlock) {
      toast(String.format(Locale.getDefault(), "DISCONNECT, %s: %s", ctrlBlock.getProductName(), ctrlBlock.getSerial()));
    }

    @Override
    public void onDetach(final UsbDevice device) {
      toast("USB_DEVICE_DETACHED");
    }

    @Override
    public void onCancel(final UsbDevice device) {
    }

    private static final String DEFAULT_USBFS = "/dev/bus/usb";

    private final String getUSBFSName(final UsbControlBlock ctrlBlock) {
      String result = null;
      final String name = ctrlBlock.getDeviceName();
      final String[] v = !TextUtils.isEmpty(name) ? name.split("/") : null;
      if ((v != null) && (v.length > 2)) {
        final StringBuilder sb = new StringBuilder(v[0]);
        for (int i = 1; i < v.length - 2; i++)
          sb.append("/").append(v[i]);
        result = sb.toString();
      }
      if (TextUtils.isEmpty(result)) {
        Timber.w("failed to get USBFS path, try to use default path: %s", name);
        result = DEFAULT_USBFS;
      }
      return result;
    }
  };

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.menu_main, menu);
    return true;
  }

  @Override
  public boolean onPrepareOptionsMenu(Menu menu) {
    if (mMynteye == null) {
      menu.findItem(R.id.action_open).setVisible(true);
      menu.findItem(R.id.action_close).setVisible(false);
    } else {
      menu.findItem(R.id.action_open).setVisible(!mMynteye.isOpened());
      menu.findItem(R.id.action_close).setVisible(mMynteye.isOpened());
    }
    menu.findItem(R.id.check_imu_data).setChecked(mImuEnabled);
    boolean featuresUsable = mMynteye != null && mMynteye.isOpened();
    menu.findItem(R.id.show_device_infos).setEnabled(featuresUsable);
    menu.findItem(R.id.show_image_params).setEnabled(featuresUsable);
    menu.findItem(R.id.show_imu_params).setEnabled(featuresUsable);
    menu.findItem(R.id.show_option_infos).setEnabled(featuresUsable);
    return super.onPrepareOptionsMenu(menu);
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    switch (item.getItemId()) {
      case R.id.action_open:
        actionOpen();
        return true;
      case R.id.action_close:
        actionClose();
        return true;
      case R.id.check_imu_data:
        mImuEnabled = !mImuEnabled;
        item.setChecked(mImuEnabled);
        return true;
      case R.id.show_device_infos:
        alert(R.string.device_infos, mMynteye.getDeviceInfos());
        return true;
      case R.id.show_image_params:
        alert(R.string.image_params, mMynteye.getImageParams());
        return true;
      case R.id.show_imu_params:
        alert(R.string.imu_params, mMynteye.getImuParams());
        return true;
      case R.id.show_option_infos:
        alert(R.string.option_infos, mMynteye.getOptionInfos());
        return true;
      default:
        return super.onOptionsItemSelected(item);
    }
  }

  private void actionOpen() {
    mTextView.setText("");
    if (mMynteye == null) {
      CameraDialog.showDialog(this);
    } else {
      mMynteye.setImuEnabled(mImuEnabled);
      mMynteye.open();
    }
  }

  private void actionClose() {
    if (mMynteye != null) {
      mMynteye.close();
      mMynteye = null;
    }
    invalidateOptionsMenu();
  }

  private void openDevice(DeviceUsbInfo info) {
    mMynteye = new Mynteye(info);
    ArrayList<StreamRequest> requests = mMynteye.getStreamRequests();
    if (requests.isEmpty()) {
      alert("Warning", "There are no streams to request :(");
      mMynteye = null;
    } else {
      ArrayList<String> items = new ArrayList<>();
      for (StreamRequest req : requests) {
        items.add(req.toString());
      }

      AlertDialog dialog = new AlertDialog.Builder(this)
          .setTitle("StreamRequests")
          .create();
      ListView listView = new ListView(this);
      listView.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, items));
      listView.setOnItemClickListener((parent, view, position, id) -> {
        dialog.dismiss();
        mMynteye.setOnStreamDataReceiveListener(this);
        mMynteye.setOnMotionDataReceiveListener(this);
        mMynteye.setImuEnabled(mImuEnabled);
        mMynteye.open(requests.get(position));
        invalidateOptionsMenu();
      });
      dialog.setOnCancelListener(dlg -> {
        mMynteye = null;
      });
      dialog.setView(listView);
      dialog.show();
    }
  }

  @Override
  public USBMonitor getUSBMonitor() {
    return mUSBMonitor;
  }

  @Override
  public void onDialogResult(boolean canceled) {
  }

  @Override
  public void onStreamDataReceive(Stream stream, StreamData data, Handler handler) {
  }

  @Override
  public void onStreamLeftReceive(StreamData data, Handler handler) {
    //Timber.i("onStreamLeftReceive");
    Frame frame = data.frame();
    if (mLeftBitmap == null) {
      mLeftBitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
    }
    BitmapUtils.copyPixels(frame, mLeftBitmap);
    mLeftImageView.post(() -> mLeftImageView.setImageBitmap(mLeftBitmap));
  }

  @Override
  public void onStreamRightReceive(StreamData data, Handler handler) {
    //Timber.i("onStreamRightReceive");
    Frame frame = data.frame();
    if (mRightBitmap == null) {
      mRightBitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
    }
    BitmapUtils.copyPixels(frame, mRightBitmap);
    mRightImageView.post(() -> mRightImageView.setImageBitmap(mRightBitmap));
  }

  @Override
  public void onMotionDataReceive(ArrayList<MotionData> datas, Handler handler) {
    if (datas.isEmpty()) return;
    ImuData data = datas.get(0).imu();
    mTextView.post(() -> {
      StringBuffer sb = new StringBuffer();
      final int flag = data.getFlag();
      if (flag == 0) {  // accel & gyro
        sb.append("Accel: ").append(data.getAccel());
        sb.append("\nGyro: ").append(data.getGyro());
      } else if (flag == 1) {  // accel
        sb.append("Accel: ").append(data.getAccel());
        sb.append("\nGyro: -");
      } else if (flag == 2) {  // gyro
        sb.append("Accel: -");
        sb.append("\nGyro: ").append(data.getGyro());
      }
      mTextView.setText(sb.toString());
    });
  }

  private void toast(int textId) {
    toast(getString(textId));
  }

  private void toast(CharSequence text) {
    Toast.makeText(this, text, Toast.LENGTH_LONG).show();
  }

  private void alert(int titleId, CharSequence message) {
    alert(getString(titleId), message);
  }

  private void alert(CharSequence title, CharSequence message) {
    new AlertDialog.Builder(this)
        .setTitle(title)
        .setMessage(message)
        .setPositiveButton(android.R.string.ok, null)
        .show();
  }
}
