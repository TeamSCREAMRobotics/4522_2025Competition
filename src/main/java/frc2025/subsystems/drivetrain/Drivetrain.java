package frc2025.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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

  public final SwerveDrivePoseEstimator globalPoseEstimate;
  public final SwerveDrivePoseEstimator specializedPoseEstimate;

  public final OdometryThread odomThread;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    globalPoseEstimate =
        new SwerveDrivePoseEstimator(
            getKinematics(), getHeading(), getState().ModulePositions, getOdometryPose());
    specializedPoseEstimate =
        new SwerveDrivePoseEstimator(
            getKinematics(), getHeading(), getState().ModulePositions, getOdometryPose());

    odomThread = new OdometryThread();
    // odomThread.start();

    helper =
        new PhoenixSwerveHelper(
            this::getGlobalPoseEstimate,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    AutoBuilder.configure(
        this::getGlobalPoseEstimate,
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

  public Pose2d getOdometryPose() {
    return getState().Pose;
  }

  public Pose2d getGlobalPoseEstimate() {
    return globalPoseEstimate.getEstimatedPosition();
  }

  public Pose2d getSpecializedPoseEstimate() {
    return specializedPoseEstimate.getEstimatedPosition();
  }

  public Rotation2d getHeading() {
    return getOdometryPose().getRotation();
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

  public BaseStatusSignal[] getAllSignals(SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    return new BaseStatusSignal[] {
      module.getDriveMotor().getPosition(),
      module.getDriveMotor().getVelocity(),
      module.getSteerMotor().getPosition(),
      module.getSteerMotor().getVelocity()
    };
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  public void logTelemetry(SwerveDriveState state) {
    Logger.log("RobotState/OdometryPose", state.Pose);
    Logger.log("Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    Logger.log("Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }

  @Override
  public void periodic() {
    globalPoseEstimate.updateWithTime(
        Timer.getFPGATimestamp(), getHeading(), getState().ModulePositions);
    /* specializedPoseEstimate.updateWithTime(
    Timer.getFPGATimestamp(), getHeading(), getState().ModulePositions); */

    if (getCurrentCommand() != null) {
      Logger.log("Subsystems/Drivetrain/ActiveCommand", getCurrentCommand().getName());
    }
    Logger.log("RobotState/EstimatedPose", globalPoseEstimate.getEstimatedPosition());
    // Logger.log("RobotState/SpecializedPoseEstimate",
    // specializedPoseEstimate.getEstimatedPosition());
  }

  private class OdometryThread extends Thread {
    private BaseStatusSignal[] m_allSignals;
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;
    public int ModuleCount = getModules().length;

    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    private double averageLoopTime = 0;

    public OdometryThread() {
      super();
      // 4 signals for each module + 2 for Pigeon2
      m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
      for (int i = 0; i < ModuleCount; ++i) {
        var signals = getAllSignals(getModule(i));
        m_allSignals[(i * 4) + 0] = signals[0];
        m_allSignals[(i * 4) + 1] = signals[1];
        m_allSignals[(i * 4) + 2] = signals[2];
        m_allSignals[(i * 4) + 3] = signals[3];
      }
      m_allSignals[m_allSignals.length - 2] = getPigeon2().getYaw();
      m_allSignals[m_allSignals.length - 1] = getPigeon2().getAngularVelocityZDevice();
    }

    @Override
    public void run() {
      /* Make sure all signals update at around 250hz */
      for (var sig : m_allSignals) {
        sig.setUpdateFrequency(250);
      }
      /* Run as fast as possible, our signals will control the timing */
      while (true) {
        /* Synchronously wait for all signals in drivetrain */
        var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
        lastTime = currentTime;
        currentTime = Utils.getCurrentTimeSeconds();
        averageLoopTime = lowpass.calculate(currentTime - lastTime);

        /* Get status of the waitForAll */
        if (status.isOK()) {
          SuccessfulDaqs++;
        } else {
          FailedDaqs++;
        }

        // Assume Pigeon2 is flat-and-level so latency compensation can be performed
        double yawDegrees =
            BaseStatusSignal.getLatencyCompensatedValue(
                    getPigeon2().getYaw(), getPigeon2().getAngularVelocityZDevice())
                .in(Units.Degrees);

        globalPoseEstimate.updateWithTime(
            Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(yawDegrees),
            Drivetrain.this.getState().ModulePositions);
        specializedPoseEstimate.updateWithTime(
            Timer.getFPGATimestamp(),
            Rotation2d.fromDegrees(yawDegrees),
            Drivetrain.this.getState().ModulePositions);
      }
    }

    public double getTime() {
      return averageLoopTime;
    }

    public int getSuccessfulDaqs() {
      return SuccessfulDaqs;
    }

    public int getFailedDaqs() {
      return FailedDaqs;
    }
  }
}
