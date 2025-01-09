package frc2025.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import drivers.TalonFXSubsystem.CANCoderConstants;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;
import util.SimUtil;

public class WristConstants {

  public static final double WRIST_REDUCTION = 30.0;
  public static final double ROLLERS_REDUCTION = 2.0;

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(1), WRIST_REDUCTION, 0.01);
  public static final ScreamPIDConstants SIM_GAINS =
      new ScreamPIDConstants(5.0 * WRIST_REDUCTION, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration WRIST_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    WRIST_CONFIG.name = "Wrist";

    WRIST_CONFIG.codeEnabled = true;
    WRIST_CONFIG.logTelemetry = false;

    WRIST_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, true);

    WRIST_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(14, ""), InvertedValue.CounterClockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    WRIST_CONFIG.cancoderConstants = new CANCoderConstants(new CANDevice(4, ""), config);

    WRIST_CONFIG.neutralMode = NeutralModeValue.Brake;
    WRIST_CONFIG.rotorToSensorRatio = WRIST_REDUCTION;
    WRIST_CONFIG.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // WRIST_CONFIG.continuousWrap = true; //TODO commented due to not being a valid configuration?
    WRIST_CONFIG.feedbackRemoteSensorId = 4;
    WRIST_CONFIG.enableSupplyCurrentLimit = true;
    WRIST_CONFIG.supplyCurrentLimit = 40;
    WRIST_CONFIG.cruiseVelocity = 20.0;
    WRIST_CONFIG.acceleration = 10.0;
    WRIST_CONFIG.slot0 =
        new ScreamPIDConstants(100.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    WRIST_CONFIG.positionThreshold = 0.025;
    ;
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.Clockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }
}
