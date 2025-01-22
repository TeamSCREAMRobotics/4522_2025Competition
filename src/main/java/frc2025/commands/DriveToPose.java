package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import util.AllianceFlipUtil;
import util.GeomUtil;

public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> targetPose;

  private final ProfiledPIDController driveController =
      DrivetrainConstants.DRIVE_ALIGNMENT_CONTROLLER;
  private final PIDController headingController = DrivetrainConstants.HEADING_CONTROLLER;

  private double driveErrorAbs;
  private Translation2d lastSetpointTranslation;

  private Optional<DoubleSupplier> yOverride = Optional.empty();
  private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    setName("DriveToPose");
  }

  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> targetPose, DoubleSupplier yOverride) {
    this(drivetrain, targetPose);
    this.yOverride = Optional.of(yOverride);
  }

  public DriveToPose(
      Drivetrain drivetrain,
      Supplier<Pose2d> targetPose,
      Supplier<Translation2d> translationOverride) {
    this(drivetrain, targetPose);
    this.translationOverride = Optional.of(translationOverride);
  }

  public DriveToPose(Drivetrain drivetrain, Pose2d targetPose) {
    this(drivetrain, () -> targetPose);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drivetrain.getFieldVelocity().dx, drivetrain.getFieldVelocity().dy)
                .rotateBy(
                    targetPose
                        .get()
                        .getTranslation()
                        .minus(drivetrain.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    headingController.reset();
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    Pose2d targetPose = this.targetPose.get();

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    driveErrorAbs = currentDistance;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double driveVelocity =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d velocity;

    if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
      velocity = translationOverride.get().get().times(AllianceFlipUtil.getDirectionCoefficient());
    } else {
      velocity =
          new Pose2d(
                  new Translation2d(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
              .getTranslation();
    }

    drivetrain.setControl(
        drivetrain
            .getHelper()
            .getApplyFieldSpeeds(
                new ChassisSpeeds(
                    velocity.getX(),
                    yOverride.isPresent() ? yOverride.get().getAsDouble() : velocity.getY(),
                    headingVelocity)));

    Logger.log("DriveToPose/MeasuredDistance", currentDistance);
    Logger.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.log("DriveToPose/MeasuredHeading", currentPose.getRotation().getDegrees());
    Logger.log("DriveToPose/SetpointHeading", targetPose.getRotation().getDegrees());
    Logger.log(
        "DriveToPose/Setpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(headingController.getSetpoint())));
    Logger.log("DriveToPose/TargetPose", targetPose);
  }

  public boolean atGoal() {
    return driveController.atGoal() && headingController.atSetpoint();
  }
}
