package frc2025.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import math.ScreamMath;
import util.AllianceFlipUtil;
import util.ScreamUtil;

public class Controlboard {

  public enum ScoringLocation {
    LEFT,
    CENTER,
    RIGHT;
  }

  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  public static final double STICK_DEADBAND = 0.05;
  public static final double TRIGGER_DEADBAND = 0.1;
  public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(4.0);

  public static DoubleSupplier elevHeightSup;

  public static boolean fieldCentric = true;

  static {
    driveController.povRight().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
  }

  public static ScoringLocation lastSide = ScoringLocation.RIGHT;

  public static Command driverRumbleCommand(Supplier<RumbleType> type, double value, double time) {
    return Commands.startEnd(
            () -> driveController.getHID().setRumble(type.get(), value),
            () -> driveController.getHID().setRumble(type.get(), 0.0))
        .withTimeout(time);
  }

  public static double applyPower(double value, int power) {
    return Math.pow(value, power) * (power % 2 == 0 ? Math.signum(value) : 1);
  }

  public static Supplier<Translation2d> getRawTranslation() {
    return () -> new Translation2d(driveController.getLeftY(), driveController.getLeftX());
  }

  public static Supplier<Translation2d> getTranslation() {
    return () ->
        snapTranslationToPole(
                new Translation2d(
                        applyPower(
                            -MathUtil.applyDeadband(driveController.getLeftY(), STICK_DEADBAND), 2),
                        applyPower(
                            -MathUtil.applyDeadband(driveController.getLeftX(), STICK_DEADBAND), 2))
                    .times(DrivetrainConstants.MAX_SPEED))
            .times(AllianceFlipUtil.getDirectionCoefficient())
            .times(
                driveController.getHID().getPOV() == 0
                    ? 0.5
                    : ScreamMath.mapRange(
                        elevHeightSup.getAsDouble(),
                        0.0,
                        ElevatorConstants.MAX_HEIGHT.getInches(),
                        1.0,
                        0.4));
  }

  public static Translation2d snapTranslationToPole(Translation2d translation) {
    if (!translation.equals(Translation2d.kZero)) {
      for (int i = 0; i < 4; i++) {
        if (ScreamUtil.withinAngleThreshold(
            Rotation2d.kCCW_90deg.times(i), translation.getAngle(), SNAP_TO_POLE_THRESHOLD)) {
          return new Translation2d(translation.getNorm(), Rotation2d.kCCW_90deg.times(i));
        }
      }
      return translation;
    } else {
      return Translation2d.kZero;
    }
  }

  public static DoubleSupplier getRotation() {
    return () ->
        applyPower(-MathUtil.applyDeadband(driveController.getRightX(), STICK_DEADBAND), 3)
            * DrivetrainConstants.MAX_ANGULAR_SPEED_RADS;
  }

  public static BooleanSupplier getFieldCentric() {
    return () -> fieldCentric;
  }

  public static BooleanSupplier getSlowMode() {
    return driveController.leftTrigger(TRIGGER_DEADBAND).or(driveController.leftBumper());
  }

  public static Trigger alternateControls() {
    return driveController.leftStick();
  }

  public static Trigger resetGyro() {
    return driveController.back();
  }

  public static Trigger startClimb() {
    return driveController.start();
  }

  public static Trigger goToLevel4() {
    return driveController.y().and(alternateControls().negate());
  }

  public static Trigger goToLevel3() {
    return driveController.x().and(alternateControls().negate());
  }

  public static Trigger goToLevel2() {
    return driveController.b().and(alternateControls().negate());
  }

  public static Trigger goToTrough() {
    return driveController.a().and(alternateControls().negate());
  }

  public static Trigger goToAlgaeClear() {
    return driveController.y().and(alternateControls());
  }

  public static Trigger groundIntake() {
    return driveController.rightTrigger(TRIGGER_DEADBAND);
  }

  public static Trigger feed() {
    return driveController.leftTrigger(TRIGGER_DEADBAND);
  }

  public static Trigger score() {
    return driveController.rightBumper();
  }

  public static Trigger alignToReef() {
    return driveController.povLeft();
  }

  public static Trigger processor() {
    return driveController.a().and(alternateControls());
  }

  public static Trigger climb() {
    return driveController.start();
  }

  public static Trigger troughFeed() {
    return driveController.leftTrigger().and(alternateControls());
  }

  public static Trigger testButton() {
    return driveController.povDown();
  }
}
