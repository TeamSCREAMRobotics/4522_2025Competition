package frc2025.subsystems.drivetrain.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import data.Length;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc2025.subsystems.drivetrain.Drivetrain;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with
  // the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(100) // 100
          .withKI(0)
          .withKD(0.2) // 0.2
          .withKS(0.1) // 0.1
          .withKV(0.25) // 2.33
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs()
          .withKP(0.1) // 0.12
          .withKI(0)
          .withKD(0) // 3
          .withKS(0)
          .withKV(0.124)
          .withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
      DriveMotorArrangement.TalonFX_Integrated;

  private static final SteerMotorArrangement STEER_MOTOR_TYPE =
      SteerMotorArrangement.TalonFX_Integrated;

  private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Current SLIP_CURRENT = Amps.of(120.0);

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final LinearVelocity SPEED_12V_MPS = MetersPerSecond.of(4.48056);

  // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double COUPLE_RATIO = 3.125;

  private static final double DRIVE_GEAR_RATIO = 7.125;
  private static final double STEER_GEAR_RATIO = 18.75;
  private static final Distance WHEEL_RADIUS = Inches.of(2); // inches

  private static final boolean STEER_INVERTED = true;

  private static final String CANBUS = "canivore";
  private static final int PIGEON_ID = 0;

  // These are only used for simulation
  private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
  private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
  // Simulated voltage necessary to overcome friction
  private static final Voltage STEER_KS = Volts.of(0.25);
  private static final Voltage DRIVE_KS = Volts.of(0.25);

  private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();

  static {
    DRIVE_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    DRIVE_CONFIG.CurrentLimits.StatorCurrentLimit = 80;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 80;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = 90;
    DRIVE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

    DRIVE_CONFIG.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
  }

  private static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();

  static {
    STEER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    STEER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    STEER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    STEER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
    STEER_CONFIG.MotionMagic.MotionMagicAcceleration = 1;
    STEER_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 2;
  }

  private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(PIGEON_ID)
          .withCANBusName(CANBUS)
          .withPigeon2Configs(new Pigeon2Configuration());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      CONSTANT_FACTORY =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
              .withSteerMotorGearRatio(STEER_GEAR_RATIO)
              .withWheelRadius(WHEEL_RADIUS)
              .withSlipCurrent(SLIP_CURRENT)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withDriveMotorType(DRIVE_MOTOR_TYPE)
              .withSteerMotorType(STEER_MOTOR_TYPE)
              .withFeedbackSource(STEER_FEEDBACK_TYPE)
              .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
              .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
              .withSpeedAt12Volts(SPEED_12V_MPS)
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_KS)
              .withDriveFrictionVoltage(DRIVE_KS)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(COUPLE_RATIO)
              .withDriveMotorInitialConfigs(DRIVE_CONFIG)
              .withSteerMotorInitialConfigs(STEER_CONFIG)
              .withEncoderInitialConfigs(new CANcoderConfiguration());

  public static final Length WHEEL_BASE = Length.fromInches(22.75); // Front to back
  public static final Length TRACK_WIDTH = Length.fromInches(20.75); // Side to side

  public static final Translation2d FRONT_LEFT_POSITION =
      new Translation2d(WHEEL_BASE.getMeters() / 2.0, TRACK_WIDTH.getMeters() / 2.0);
  public static final Translation2d FRONT_RIGHT_POSITION =
      new Translation2d(WHEEL_BASE.getMeters() / 2.0, -TRACK_WIDTH.getMeters() / 2.0);
  public static final Translation2d BACK_LEFT_POSITION =
      new Translation2d(-WHEEL_BASE.getMeters() / 2.0, TRACK_WIDTH.getMeters() / 2.0);
  public static final Translation2d BACK_RIGHT_POSITION =
      new Translation2d(-WHEEL_BASE.getMeters() / 2.0, -TRACK_WIDTH.getMeters() / 2.0);

  public static final double DRIVE_BASE_RADIUS = FRONT_LEFT_POSITION.getNorm();

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      MODULE_0 =
          CONSTANT_FACTORY.createModuleConstants(
              0, 1, 0, -0.409912109375, 0, 0, false, STEER_INVERTED, false);

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      MODULE_1 =
          CONSTANT_FACTORY.createModuleConstants(
              2, 3, 1, 0.019775390625, 0, 0, true, STEER_INVERTED, false);

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      MODULE_2 =
          CONSTANT_FACTORY.createModuleConstants(
              4, 5, 2, 0.368408203125, 0, 0, false, STEER_INVERTED, false);

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      MODULE_3 =
          CONSTANT_FACTORY.createModuleConstants(
              6, 7, 3, -0.29736328125, 0, 0, true, STEER_INVERTED, false);

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          MODULE_0
              .withLocationX(FRONT_LEFT_POSITION.getX())
              .withLocationY(FRONT_LEFT_POSITION.getY());
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          MODULE_1
              .withLocationX(FRONT_RIGHT_POSITION.getX())
              .withLocationY(FRONT_RIGHT_POSITION.getY());
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          MODULE_2
              .withLocationX(BACK_LEFT_POSITION.getX())
              .withLocationY(BACK_LEFT_POSITION.getY());
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          MODULE_3
              .withLocationX(BACK_RIGHT_POSITION.getX())
              .withLocationY(BACK_RIGHT_POSITION.getY());

  private static final double ODOMETRY_UPDATE_FREQUENCY = 250.0;

  public static final Drivetrain DriveTrain =
      new Drivetrain(
          DRIVETRAIN_CONSTANTS,
          ODOMETRY_UPDATE_FREQUENCY,
          FRONT_LEFT,
          FRONT_RIGHT,
          BACK_LEFT,
          BACK_RIGHT);

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
