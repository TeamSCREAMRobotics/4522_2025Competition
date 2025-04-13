package frc2025.subsystems.climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import drivers.TalonFXSubsystem.CANCoderConstants;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;

public class ClimberConstants {

  public static final double REDUCTION = 81;

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Climber";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.CounterClockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
    config.MagnetSensor.MagnetOffset = 0.008056640625;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    CONFIGURATION.cancoderConstants = new CANCoderConstants(new CANDevice(5), config);

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = 1.0;

    CONFIGURATION.rotorToSensorRatio = REDUCTION;
    CONFIGURATION.feedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    CONFIGURATION.feedbackRemoteSensorId = 5;
    // CONFIGURATION.feedbackRotorOffset = 0.016845703125;

    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.cruiseVelocity = 15.0; // 8.0
    CONFIGURATION.acceleration = 10.0; // 4.0
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(140.0, 0, 0).getSlot0Configs(new FeedforwardConstants()); // 120
    CONFIGURATION.positionThreshold = 0.005;
  }
}
