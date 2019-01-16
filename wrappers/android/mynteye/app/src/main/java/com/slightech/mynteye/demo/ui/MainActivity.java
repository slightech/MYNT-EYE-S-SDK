package com.slightech.mynteye.demo.ui;

import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.Toast;
import androidx.appcompat.app.AlertDialog;
import butterknife.ButterKnife;
import com.slightech.mynteye.Device;
import com.slightech.mynteye.DeviceUsbInfo;
import com.slightech.mynteye.demo.R;
import com.slightech.mynteye.demo.util.RootUtils;
import java.util.ArrayList;
import java.util.Locale;

public class MainActivity extends BaseActivity {

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
