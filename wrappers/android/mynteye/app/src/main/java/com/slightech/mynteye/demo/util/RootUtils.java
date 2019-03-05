package com.slightech.mynteye.demo.util;

import com.stericson.RootShell.RootShell;
import com.stericson.RootShell.exceptions.RootDeniedException;
import com.stericson.RootShell.execution.Command;
import com.stericson.RootShell.execution.Shell;
import java.io.IOException;
import java.util.concurrent.TimeoutException;
import timber.log.Timber;

public final class RootUtils {

  public interface OnRequestAccessibleListener {
    void onRequestAccessible(boolean ok);
  }

  public static boolean isRooted() {
    if (!RootShell.isRootAvailable()) {
      Timber.e("Root not found");
      return false;
    }

    try {
      RootShell.getShell(true);
    } catch (IOException e) {
      e.printStackTrace();
      return false;
    } catch (TimeoutException e) {
      Timber.e("TIMEOUT EXCEPTION!");
      e.printStackTrace();
      return false;
    } catch (RootDeniedException e) {
      Timber.e("ROOT DENIED EXCEPTION!");
      e.printStackTrace();
      return false;
    }

    try {
      if (!RootShell.isAccessGiven()) {
        Timber.e("ERROR: No root access to this device.");
        return false;
      }
    } catch (Exception e) {
      Timber.e("ERROR: could not determine root access to this device.");
      return false;
    }

    return true;
  }

  public static void requestAccessible(OnRequestAccessibleListener l) {
    try {
      Shell sh = RootShell.getShell(true);
      sh.add(new Command(1, "chmod 666 /dev/video*") {
        @Override
        public void commandOutput(int id, String line) {
          Timber.d("commandOutput: %s", line);
          super.commandOutput(id, line);
        }
        @Override
        public void commandTerminated(int id, String reason) {
          Timber.d("commandTerminated: %s", reason);
        }
        @Override
        public void commandCompleted(int id, int exitcode) {
          Timber.d("commandCompleted: %s", ((exitcode == 0) ? "ok" : "fail"));
          if (l != null) l.onRequestAccessible(exitcode == 0);
        }
      });
      sh.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

}
