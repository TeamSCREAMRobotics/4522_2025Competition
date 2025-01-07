package frc2025.logging;

import dev.doglog.DogLog;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger extends DogLog {
  private static boolean debug = false;

  public static void enableDebug(boolean enable) {
    debug = enable;
  }

  public static void log(String key, Mechanism2d value) {
    SmartDashboard.putData(key, value);
  }

  public static void logDebug(String key, Mechanism2d value) {
    if (debug) {
      SmartDashboard.putData(key, value);
    }
  }

  public static void logDebug(String key, boolean... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, double... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, float... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, int... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, long... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, String... value) {
    if (debug) {
      log(key, value);
    }
  }

  public static void logDebug(String key, Enum<?>... value) {
    if (debug) {
      log(key, value);
    }
  }

  @SuppressWarnings("unchecked")
  public static <T extends StructSerializable> void logDebug(String key, T... value) {
    if (debug) {
      log(key, value);
    }
  }
}
