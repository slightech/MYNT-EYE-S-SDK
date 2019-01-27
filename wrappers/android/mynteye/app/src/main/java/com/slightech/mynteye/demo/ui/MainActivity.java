package com.slightech.mynteye.demo.ui;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
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
import com.slightech.mynteye.Device;
import com.slightech.mynteye.DeviceUsbInfo;
import com.slightech.mynteye.Frame;
import com.slightech.mynteye.MotionData;
import com.slightech.mynteye.Stream;
import com.slightech.mynteye.StreamData;
import com.slightech.mynteye.StreamRequest;
import com.slightech.mynteye.demo.R;
import com.slightech.mynteye.demo.camera.Mynteye;
import com.slightech.mynteye.demo.util.RootUtils;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Locale;
import timber.log.Timber;

public class MainActivity extends BaseActivity implements Mynteye.OnStreamDataReceiveListener,
    Mynteye.OnMotionDataReceiveListener{

  @BindView(R.id.text) TextView mTextView;
  @BindView(R.id.image_left) ImageView mLeftImageView;
  @BindView(R.id.image_right) ImageView mRightImageView;

  private Mynteye mMynteye;
  private Bitmap mLeftBitmap, mRightBitmap;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    ButterKnife.bind(this);
  }

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.menu_main, menu);
    MenuItem item = menu.findItem(R.id.action_open);
    if (item != null) {
      item.setEnabled(false);
      actionOpen(() -> item.setEnabled(true));
    }
    return true;
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    switch (item.getItemId()) {
      case R.id.action_open:
        item.setEnabled(false);
        actionOpen(() -> item.setEnabled(true));
        return true;
      default:
        return super.onOptionsItemSelected(item);
    }
  }

  private void actionOpen(final Runnable completeEvent) {
    if (!RootUtils.isRooted()) {
      if (completeEvent != null) completeEvent.run();
      alert("Warning", "Root denied :(");
      return;
    }
    RootUtils.requestAccessible(ok -> {
      if (completeEvent != null) completeEvent.run();
      if (ok) {
        toast("Root granted :)");
        showDevices();
      } else {
        alert("Warning", "There are no devices accessible.");
      }
    });
  }

  private void showDevices() {
    ArrayList<DeviceUsbInfo> infos = Device.query();
    if (infos.isEmpty()) {
      alert("Warning", "There are no devices :(");
    } else {
      ArrayList<String> items = new ArrayList<>();
      for (DeviceUsbInfo info : infos) {
        items.add(String.format(Locale.getDefault(), "%d, %s, SN: %s",
            info.getIndex(), info.getName(), info.getSn()));
      }

      AlertDialog dialog = new AlertDialog.Builder(this)
          .setTitle("Devices")
          .create();
      ListView listView = new ListView(this);
      listView.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_list_item_1, items));
      listView.setOnItemClickListener((parent, view, position, id) -> {
        dialog.dismiss();
        openDevice(infos.get(position));
      });
      dialog.setView(listView);
      dialog.show();
    }
  }

  private void openDevice(DeviceUsbInfo info) {
    mMynteye = new Mynteye(info);
    ArrayList<StreamRequest> requests = mMynteye.getStreamRequests();
    if (requests.isEmpty()) {
      alert("Warning", "There are no streams to request :(");
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
        mMynteye.open(requests.get(position));
      });
      dialog.setView(listView);
      dialog.show();
    }
  }

  @Override
  public void onStreamDataReceive(Stream stream, StreamData data, Handler handler) {
  }

  @Override
  public void onStreamLeftReceive(StreamData data, Handler handler) {
    //Timber.i("onStreamLeftReceive");
    Frame frame = data.frame();
    if (mLeftBitmap == null) {
      Bitmap.Config config;
      switch (frame.format()) {
        case GREY: config = Bitmap.Config.ALPHA_8; break;
        case RGB888: config = Bitmap.Config.ARGB_8888; break;
        default: Timber.e("Unaccepted stream format"); return;
      }
      mLeftBitmap = Bitmap.createBitmap(frame.width(), frame.height(), config);
    }
    mLeftBitmap.copyPixelsFromBuffer(ByteBuffer.wrap(frame.data()));
    mLeftImageView.post(() -> mLeftImageView.setImageBitmap(mLeftBitmap));
  }

  @Override
  public void onStreamRightReceive(StreamData data, Handler handler) {
    //Timber.i("onStreamRightReceive");
    Frame frame = data.frame();
    if (mRightBitmap == null) {
      Bitmap.Config config;
      switch (frame.format()) {
        case GREY: config = Bitmap.Config.ALPHA_8; break;
        case RGB888: config = Bitmap.Config.ARGB_8888; break;
        default: Timber.e("Unaccepted stream format"); return;
      }
      mRightBitmap = Bitmap.createBitmap(frame.width(), frame.height(), config);
    }
    mRightBitmap.copyPixelsFromBuffer(ByteBuffer.wrap(frame.data()));
    mRightImageView.post(() -> mRightImageView.setImageBitmap(mRightBitmap));
  }

  @Override
  public void onMotionDataReceive(ArrayList<MotionData> datas, Handler handler) {
    if (datas.isEmpty()) return;
    mTextView.post(() -> mTextView.setText(datas.get(0).imu().toString()));
  }

  @Override
  protected void onDestroy() {
    if (mMynteye != null) {
      mMynteye.close();
      mMynteye = null;
    }
    super.onDestroy();
  }

  private void toast(CharSequence text) {
    Toast.makeText(this, text, Toast.LENGTH_LONG).show();
  }

  private void alert(CharSequence title, CharSequence message) {
    new AlertDialog.Builder(this)
        .setTitle(title)
        .setMessage(message)
        .setPositiveButton(android.R.string.ok, null)
        .show();
  }
}
