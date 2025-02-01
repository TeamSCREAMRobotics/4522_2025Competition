package frc2025.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc2025.subsystems.superstructure.wrist.WristConstants;

public class ComponentVisualizer {

  public static final Pose3d INTAKE_ORIGIN_POSE =
      new Pose3d(0.3175, -0.3175, 0.212725, new Rotation3d());

  public static final Pose3d WRIST_ORIGIN_POSE =
      new Pose3d(0.0, 0.048933, 0.217886, new Rotation3d());

  public static Pose3d getIntakePose(Rotation2d deployAngle) {
    return new Pose3d(
        INTAKE_ORIGIN_POSE.getX(),
        INTAKE_ORIGIN_POSE.getY(),
        INTAKE_ORIGIN_POSE.getZ(),
        new Rotation3d(0, deployAngle.getRadians(), 0));
  }

  public static Pose3d getStage1Pose(double elevatorHeight) {
    return new Pose3d(
        0.0,
        0.0,
        MathUtil.clamp(elevatorHeight, 1.329054, 1.437005 * 2) - 1.329054,
        new Rotation3d());
  }

  public static Pose3d getStage2Pose(double elevatorHeight) {
    return new Pose3d(
        0.0,
        0.0,
        MathUtil.clamp(elevatorHeight, 0.610552, 1.437005 * 2) - 0.610552,
        new Rotation3d());
  }

  public static Pose3d getCarriagePose(double elevatorHeight) {
    return new Pose3d(0.0, 0.0, elevatorHeight, new Rotation3d());
  }

  public static Pose3d getWristPose(double elevatorHeight, Rotation2d wristAngle) {
    return new Pose3d(
        WRIST_ORIGIN_POSE.getX(),
        WRIST_ORIGIN_POSE.getY(),
        elevatorHeight + WRIST_ORIGIN_POSE.getZ(),
        new Rotation3d(0, wristAngle.getRadians(), 0).unaryMinus());
  }

  public static Pose3d getCoralPose(double elevatorHeight, Rotation2d wristAngle) {
    Pose3d wristPose = getWristPose(elevatorHeight, wristAngle);
    Translation2d relPos =
        new Translation2d(Units.inchesToMeters((11.93 / 2.0) + 6.737390), wristAngle)
            .plus(new Translation2d(0, wristPose.getZ()));
    return new Pose3d(
        relPos.getX(),
        WristConstants.ROLLERS_TO_ORIGIN.getMeters(),
        relPos.getY(),
        new Rotation3d(0, -wristAngle.getRadians(), 0));
  }

  public static Pose3d getAlgaePose(double elevatorHeight, Rotation2d wristAngle) {
    Pose3d wristPose = getWristPose(elevatorHeight, wristAngle);
    Translation2d relPos =
        new Translation2d(Units.inchesToMeters(17.426), wristAngle.plus(new Rotation2d(0.4614)))
            .plus(new Translation2d(0, wristPose.getZ()));
    return new Pose3d(
        relPos.getX(),
        WristConstants.ROLLERS_TO_ORIGIN.getMeters(),
        relPos.getY(),
        new Rotation3d());
  }
}
