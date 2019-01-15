package com.slightech.mynteye.demo;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import com.slightech.mynteye.Device;
import com.slightech.mynteye.DeviceUsbInfo;
import timber.log.Timber;

public class MainActivity extends AppCompatActivity {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    for (DeviceUsbInfo info : Device.query()) {
      Timber.i(info.toString());
    }
  }
}
