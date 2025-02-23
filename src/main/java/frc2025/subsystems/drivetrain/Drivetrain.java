package frc2025.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.Robot;
import frc2025.constants.Constants;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;
import lombok.Getter;
import util.RunnableUtil.RunOnce;
import util.ScreamUtil;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private Notifier simThread = null;
  private double lastSimTime;

  private RunOnce operatorPerspectiveApplier = new RunOnce();

  @Getter private final PhoenixSwerveHelper helper;

  /* public final SwerveDrivePoseEstimator globalPoseEstimate;
  public final SwerveDrivePoseEstimator specializedPoseEstimate; */

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    /* globalPoseEstimate =
        new SwerveDrivePoseEstimator(
            getKinematics(), getHeading(), getState().ModulePositions, getOdometryPose());
    specializedPoseEstimate =
        new SwerveDrivePoseEstimator(
            getKinematics(), getHeading(), getState().ModulePositions, getOdometryPose()); */

    helper =
        new PhoenixSwerveHelper(
            this::getEstimatedPose,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    AutoBuilder.configure(
        this::getEstimatedPose,
        this::resetPose,
        () -> getState().Speeds,
        (speeds, feedforwards) ->
            setControl(
                helper
                    .getApplyRobotSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        DrivetrainConstants.PATH_FOLLOWING_CONTROLLER,
        DrivetrainConstants.ROBOT_CONFIG,
        () -> false,
        this);

    registerTelemetry(this::logTelemetry);

    if (Robot.isSimulation()) {
      startSimThread();
    }

    System.out.println("[Init] Drivetrain initialization complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void startSimThread() {
    simThread =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simThread.startPeriodic(Constants.SIM_PERIOD_SEC);
  }

  public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold) {
    return ScreamUtil.withinAngleThreshold(targetAngle, getHeading(), threshold);
  }

  public Pose2d getEstimatedPose() {
    return getState().Pose;
  }

  /* public Pose2d getGlobalPoseEstimate() {
    return globalPoseEstimate.getEstimatedPosition();
  }

  public Pose2d getSpecializedPoseEstimate() {
    return specializedPoseEstimate.getEstimatedPosition();
  } */

  public Rotation2d getHeading() {
    return getEstimatedPose().getRotation();
  }

  public Rotation2d getYawRate() {
    return Rotation2d.fromDegrees(
        getPigeon2().getAngularVelocityZWorld().asSupplier().get().in(Units.DegreesPerSecond));
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    for (int i = 0; i < DrivetrainConstants.NUM_MODULES; i++) {
      states[i] = getModules()[i].getCurrentState();
    }
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  public Translation2d getLinearVelocity() {
    return new Translation2d(
            getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond)
        .rotateBy(getHeading());
  }

  public Twist2d getFieldVelocity() {
    return new Twist2d(
        getLinearVelocity().getX(),
        getLinearVelocity().getY(),
        getState().Speeds.omegaRadiansPerSecond);
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    helper.setLastAngle(rotation);
    super.resetRotation(rotation);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Logger.log("Vision/ActiveVisionMeasurement", visionRobotPoseMeters);
    // Logger.log("Vision/ActiveStdDevs", visionMeasurementStdDevs);
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  public void logTelemetry(SwerveDriveState state) {
    Logger.log("RobotState/EstimatedPose", state.Pose); /*
    Logger.log("Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    Logger.log("Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);

    Dashboard.publishPose(getEstimatedPose()); */
  }

  @Override
  public void periodic() {
    if (getCurrentCommand() != null) {
      Logger.log("Subsystems/Drivetrain/ActiveCommand", getCurrentCommand().getName());
    }
    // Logger.log("RobotState/EstimatedPose", globalPoseEstimate.getEstimatedPosition());
    // Logger.log("RobotState/SpecializedPoseEstimate",
    // specializedPoseEstimate.getEstimatedPosition());
  }
}
