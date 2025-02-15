package frc2025.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.constants.FieldConstants;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import vision.LimelightHelpers;
import vision.LimelightHelpers.PoseEstimate;
import vision.LimelightVision.Limelight;

public class VisionManager extends SubsystemBase {

  public static class Limelights {
    public static final Limelight REEF_LEFT =
        new Limelight(
            "limelight-reefB",
            new Pose3d(
                Units.inchesToMeters(8.194093),
                Units.inchesToMeters(10.687941),
                Units.inchesToMeters(8.032948),
                new Rotation3d(0, Units.degreesToRadians(28.1), -Units.degreesToRadians(30))));
    public static final Limelight REEF_RIGHT =
        new Limelight(
            "limelight-reefA",
            new Pose3d(
                Units.inchesToMeters(8.194093),
                -Units.inchesToMeters(10.687941),
                Units.inchesToMeters(8.032948),
                new Rotation3d(0, Units.degreesToRadians(28.1), Units.degreesToRadians(30))));
    public static final Limelight FRONT_ELEVATOR =
        new Limelight(
            "limelight-front",
            new Pose3d(
                Units.inchesToMeters(6.223684),
                Units.inchesToMeters(11.834409),
                Units.inchesToMeters(9.971965),
                new Rotation3d(Units.degreesToRadians(90.0), 0.0, -Units.degreesToRadians(30))));
    public static final Limelight STATION_ELEVATOR =
        new Limelight(
            "limelight-station",
            new Pose3d(
                -Units.inchesToMeters(1.492570),
                0,
                Units.inchesToMeters(40.156036),
                new Rotation3d(0, -Units.degreesToRadians(30.0), 0)));
  }

  private final Drivetrain drivetrain;
  private final Limelight[] limelights =
      new Limelight[] {
        Limelights.REEF_LEFT,
        Limelights.REEF_RIGHT,
        Limelights.FRONT_ELEVATOR,
        Limelights.STATION_ELEVATOR
      };

  public VisionManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public record VisionMeasurement(PoseEstimate estimate, Matrix<N3, N1> stdDevs) {}

  private void addGlobalPoseEstimate(Limelight limelight) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);
    PoseEstimate mtEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());
    PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());

    if (isValidEstimate(mtEstimate)) {
      Logger.log("Vision/" + limelight.name() + "/MegaTagEstimate", mtEstimate.pose);
      if (DriverStation.isDisabled()) {
        drivetrain.resetPose(mtEstimate.pose);
      }
    }

    if (isValidEstimate(mt2Estimate)) {
      Logger.log("Vision/" + limelight.name() + "/MegaTag2Estimate", mt2Estimate.pose);
      drivetrain.globalPoseEstimate.addVisionMeasurement(
          mt2Estimate.pose,
          mt2Estimate.timestampSeconds,
          VecBuilder.fill(
              Math.pow(0.5, mt2Estimate.tagCount) * mt2Estimate.avgTagDist * 2,
              Math.pow(0.5, mt2Estimate.tagCount) * mt2Estimate.avgTagDist * 2,
              9999999));
    }
  }

  private void addSpecializedPoseEstimate(Limelight limelight) {
    PoseEstimate mtEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());
  }

  private boolean isValidEstimate(PoseEstimate estimate) {
    return estimate != null
        && estimate.tagCount != 0
        && FieldConstants.FIELD_AREA.contains(estimate.pose)
        && !(Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 540)
        && !(drivetrain.getLinearVelocity().getNorm() > 3.0);
  }

  @Override
  public void periodic() {
    for (Limelight ll : limelights) {
      addGlobalPoseEstimate(ll);
    }
  }
}
