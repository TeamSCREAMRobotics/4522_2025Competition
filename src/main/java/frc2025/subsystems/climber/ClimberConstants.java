package frc2025.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;

public class ClimberConstants {

  public static final double REDUCTION = 556.875; // 201.6

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Climber";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.masterConstants =
    new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive); // Right Climber

    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive) // Left Climber
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.cruiseVelocity = 0.35; // 1.25
    CONFIGURATION.acceleration = 0.75; // 0.75
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(90.0, 0, 0).getSlot0Configs(new FeedforwardConstants()); // 75
    CONFIGURATION.positionThreshold = 0.005;
  }
}
