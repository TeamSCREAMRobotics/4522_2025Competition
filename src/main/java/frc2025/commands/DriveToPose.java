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
import frc2025.RobotContainer.Subsystems;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import frc2025.subsystems.superstructure.elevator.Elevator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import util.GeomUtil;

public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final Elevator elevator;
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

  public DriveToPose(Subsystems subsystems, Supplier<Pose2d> targetPose) {
    this.drivetrain = subsystems.drivetrain();
    this.elevator = subsystems.superstructure().getElevator();
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    setName("DriveToPose");
    driveController.setTolerance(0.01);
    headingController.setTolerance(Units.degreesToRadians(1));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public DriveToPose(Subsystems subsystems, Supplier<Pose2d> targetPose, DoubleSupplier yOverride) {
    this(subsystems, targetPose);
    this.yOverride = Optional.of(yOverride);
  }

  public DriveToPose(
      Subsystems subsystems,
      Supplier<Pose2d> targetPose,
      Supplier<Translation2d> translationOverride,
      BooleanSupplier useSpecializedPoseEstimate) {
    this(subsystems, targetPose);
    this.translationOverride = Optional.of(translationOverride);
  }

  public DriveToPose(Subsystems subsystems, Pose2d targetPose) {
    this(subsystems, () -> targetPose);
  }

  @Override
  public void initialize() {
    firstRun = true;
    Pose2d currentPose = drivetrain.getEstimatedPose();
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
    if (((yOverride.isPresent() && Math.abs(targetPose.minus(currentPose).getX()) < 0.03)
            || (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.03)
                && !(translationOverride.isPresent()
                    && translationOverride.get().get().getNorm() > 0.5))
        && firstRun) {
      return;
    }

    if(yOverride.isPresent() && Math.abs(targetPose.minus(currentPose).getX()) < 0.03){
      return;
    } else if(currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.03){
      return;
    } else if (!(translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5)){
      return;
    } 

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScalar = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    double elevHeightScalar;
    if (elevator.getMeasuredHeight().getInches() == 0.0) {
      elevHeightScalar = 1.0;
    } else {
      elevHeightScalar =
          MathUtil.clamp(((1.0 / elevator.getMeasuredHeight().getInches()) * 30.0), 0.0, 1.0);
    }

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
        driveController.getSetpoint().velocity * ffScalar
            + driveController.calculate(driveErrorAbs, 0.0) * elevHeightScalar;
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d velocity;

    if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
      velocity = translationOverride.get().get();
    } else if(yOverride.isPresent()){
      velocity = new Translation2d(driveVelocity, yOverride.get().getAsDouble());
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
