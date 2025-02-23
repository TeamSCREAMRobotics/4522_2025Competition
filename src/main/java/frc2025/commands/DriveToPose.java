package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import util.GeomUtil;

public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> targetPose;

  private final ProfiledPIDController driveController =
      DrivetrainConstants.DRIVE_ALIGNMENT_CONTROLLER;
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(5.0, 0, 0, new Constraints(4, Units.degreesToRadians(720)));

  private double driveErrorAbs;
  private Translation2d lastSetpointTranslation;

  private Optional<DoubleSupplier> yOverride = Optional.empty();
  private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();

  private boolean firstRun;

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
      Supplier<Translation2d> translationOverride,
      BooleanSupplier useSpecializedPoseEstimate) {
    this(drivetrain, targetPose);
    this.translationOverride = Optional.of(translationOverride);
  }

  public DriveToPose(Drivetrain drivetrain, Pose2d targetPose) {
    this(drivetrain, () -> targetPose);
  }

  @Override
  public void initialize() {
    firstRun = true;
    Pose2d currentPose = drivetrain.getEstimatedPose();
    driveController.setTolerance(0.01);
    headingController.setTolerance(Units.degreesToRadians(1));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drivetrain.getFieldVelocity().dx, drivetrain.getFieldVelocity().dy)
                .rotateBy(
                    targetPose
                        .get()
                        .getTranslation()
                        .minus(drivetrain.getEstimatedPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    headingController.reset(
        currentPose.getRotation().getRadians(), drivetrain.getYawRate().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getEstimatedPose();
    Pose2d targetPose = this.targetPose.get();
    if (((yOverride.isPresent() && Math.abs(targetPose.minus(currentPose).getX()) < 0.1)
            || (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1)
                && !(translationOverride.isPresent()
                    && translationOverride.get().get().getNorm() > 0.5))
        && firstRun) {
      return;
    }

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

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocity =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d velocity;

    if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
      velocity = translationOverride.get().get();
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

    firstRun = false;

    Logger.log("DriveToPose/MeasuredDistance", currentDistance);
    Logger.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.log("DriveToPose/MeasuredHeading", currentPose.getRotation().getDegrees());
    Logger.log("DriveToPose/SetpointHeading", targetPose.getRotation().getDegrees());
    Logger.log(
        "DriveToPose/Setpoint",
        new Pose2d(
            lastSetpointTranslation, new Rotation2d(headingController.getSetpoint().position)));
    Logger.log("DriveToPose/TargetPose", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && headingController.atGoal();
  }
}
