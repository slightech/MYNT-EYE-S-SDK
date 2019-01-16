package com.slightech.mynteye.demo.ui;

import android.annotation.SuppressLint;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import static android.Manifest.permission.CAMERA;
import static android.Manifest.permission.WRITE_EXTERNAL_STORAGE;

@SuppressLint("Registered")
public class BaseActivity extends AppCompatActivity {

  private final int REQ_PERMISSIONS = 1;

  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    requestPermissions();
  }

  private void requestPermissions() {
    final String[] permissions = new String[]{WRITE_EXTERNAL_STORAGE, CAMERA};

    boolean granted = true;
    for (String permission : permissions) {
      if (ContextCompat.checkSelfPermission(this, permission)
          != PackageManager.PERMISSION_GRANTED) {
        granted = false;
      }
    }
    if (granted) return;

    ActivityCompat.requestPermissions(this, permissions, REQ_PERMISSIONS);
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
      @NonNull int[] grantResults) {
    if (requestCode == REQ_PERMISSIONS) {
      boolean granted = true;
      if (grantResults.length < 1) {
        granted = false;
      } else {
        for (int result : grantResults) {
          if (result != PackageManager.PERMISSION_GRANTED) {
            granted = false;
          }
        }
      }
      if (!granted) {
        Toast.makeText(this, "Permission denied :(", Toast.LENGTH_LONG).show();
      }
    } else {
      super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }
  }
}
