package frc2025.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;

public class ClimberConstants {

  public static final double REDUCTION = 198.0;

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Climber";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.CounterClockwise_Positive);

    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive)
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Coast;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.cruiseVelocity = 20.0;
    CONFIGURATION.acceleration = 10.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(25.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    CONFIGURATION.positionThreshold = edu.wpi.first.math.util.Units.degreesToRotations(2.0);
  }
}
