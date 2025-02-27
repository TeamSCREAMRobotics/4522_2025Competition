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
import frc2025.controlboard.Controlboard;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import math.ScreamMath;
import util.GeomUtil;

public class DriveToPose extends Command {

  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Supplier<Pose2d> targetPose;

  private final ProfiledPIDController driveController =
      DrivetrainConstants.DRIVE_ALIGNMENT_CONTROLLER;
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(5.0, 0, 0, new Constraints(4, Units.degreesToRadians(720)));

  public static double driveErrorAbs;
  private Translation2d lastSetpointTranslation;

  private Optional<DoubleSupplier> yOverride = Optional.empty();
  private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();

  private BooleanSupplier slowMode = () -> false;

  private final double driveTolerance = 0.01;

  public DriveToPose(Subsystems subsystems, Supplier<Pose2d> targetPose) {
    this.drivetrain = subsystems.drivetrain();
    this.elevator = subsystems.superstructure().getElevator();
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    setName("DriveToPose");
    driveController.setTolerance(driveTolerance);
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
      BooleanSupplier slowMode) {
    this(subsystems, targetPose);
    this.translationOverride = Optional.of(translationOverride);
    this.slowMode = slowMode;
  }

  public DriveToPose(Subsystems subsystems, Pose2d targetPose) {
    this(subsystems, () -> targetPose);
  }

  @Override
  public void initialize() {
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

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScalar = MathUtil.clamp((currentDistance) / (0.8), 0.0, 1.0);
    double elevHeightScalar = elevator.getMeasuredHeight().getInches();
    elevHeightScalar =
        MathUtil.clamp(elevHeightScalar, 0.0, ElevatorConstants.MAX_HEIGHT.getInches());
    elevHeightScalar =
        ScreamMath.mapRange(
            elevHeightScalar, 0.0, ElevatorConstants.MAX_HEIGHT.getInches(), 1.0, 0.75);

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
    } else if (yOverride.isPresent()) {
      velocity =
          new Pose2d(
                  new Translation2d(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
              .getTranslation();
      velocity = new Translation2d(velocity.getX(), yOverride.get().getAsDouble());
    } else {
      velocity =
          new Pose2d(
                  new Translation2d(),
                  currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
              .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
              .getTranslation();
    }

    if (!(translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5)) {
      if (yOverride.isPresent()
          && Math.abs(targetPose.minus(currentPose).getX()) < driveTolerance) {
        velocity = new Translation2d(0.0, yOverride.get().getAsDouble());
      } else if (Math.abs(targetPose.getTranslation().minus(currentPose.getTranslation()).getX())
              < driveTolerance
          && Math.abs(targetPose.getTranslation().minus(currentPose.getTranslation()).getY())
              < driveTolerance) {
        Controlboard.isSwitchingSide = false;
        velocity = new Translation2d();
      }
    }

    velocity = velocity.times(elevHeightScalar);

    if (slowMode.getAsBoolean()) {
      GeomUtil.normalize(velocity);
    }

    drivetrain.setControl(
        drivetrain
            .getHelper()
            .getApplyFieldSpeeds(
                new ChassisSpeeds(velocity.getX(), velocity.getY(), headingVelocity)));

    Logger.log("DriveToPose/FFScalar", ffScalar);
    Logger.log("DriveToPose/HeightScalar", elevHeightScalar);
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

  /* @Override
  public boolean isFinished() {
    return driveController.atGoal() && headingController.atGoal();
  } */
}
