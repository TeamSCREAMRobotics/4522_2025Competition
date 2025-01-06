// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.constants;

public final class Constants {
  public static final double PERIOD_SEC = 0.02;
  public static final double PERIOD_HZ = 1 / PERIOD_SEC;
  public static final double SIM_PERIOD_SEC = 0.005;

  public static final Mode ROBOT_MODE = Mode.SIM;

  public static enum Mode {
    REAL,
    SIM;
  }
}
