package frc2025.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import data.Length;
import drivers.TalonFXSubsystem.CANCoderConstants;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import math.ScreamMath;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public class WristConstants {

  public static final Length ROLLERS_TO_ORIGIN = Length.fromInches(1.383);
  public static final Length MANIPULATOR_LENGTH = Length.fromInches(18.668462);

  public static final double WRIST_REDUCTION = 30.0;
  public static final double ROLLERS_REDUCTION = 2.25;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          WRIST_REDUCTION,
          ScreamMath.parallelAxisTheorem(
                      KilogramSquareMeters.of(0.0951567339),
                      Pounds.of(4.6028013),
                      Length.fromInches(12.024525))
                  .in(KilogramSquareMeters)
              - 0.002,
          Units.inchesToMeters(20.5),
          Units.rotationsToRadians(-9999),
          Units.rotationsToRadians(9999),
          true,
          0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(600.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration WRIST_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    WRIST_CONFIG.name = "Wrist";

    WRIST_CONFIG.codeEnabled = true;
    WRIST_CONFIG.logTelemetry = false;
    WRIST_CONFIG.debugMode = false;

    WRIST_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            SIM_GAINS.getProfiledPIDController(new Constraints(100.0, 75.0), -0.5, 0.5),
            false,
            false,
            false);

    WRIST_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(14), InvertedValue.Clockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    WRIST_CONFIG.cancoderConstants = new CANCoderConstants(new CANDevice(4), config);

    WRIST_CONFIG.neutralMode = NeutralModeValue.Brake;
    WRIST_CONFIG.rotorToSensorRatio = WRIST_REDUCTION;
    WRIST_CONFIG.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    WRIST_CONFIG.continuousWrap = true;
    WRIST_CONFIG.feedbackRemoteSensorId = 4;
    WRIST_CONFIG.enableSupplyCurrentLimit = true;
    WRIST_CONFIG.supplyCurrentLimit = 40;
    WRIST_CONFIG.cruiseVelocity = 1.0;
    WRIST_CONFIG.acceleration = 1.0;
    WRIST_CONFIG.slot0 =
        new ScreamPIDConstants(1.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    WRIST_CONFIG.positionThreshold = Units.degreesToRotations(15);
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "WristRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(15), InvertedValue.CounterClockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }
}
