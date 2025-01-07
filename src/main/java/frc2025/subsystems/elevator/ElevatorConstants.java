package frc2025.subsystems.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public final class ElevatorConstants {

  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT = Length.fromFeet(6.0);
  public static final double ENCODER_MAX = 10.37;
  public static final double ENCODER_MIN = 0.0;
  public static final Length PULLEY_DIAMETER = Length.fromInches(2.211);
  public static final Length PULLEY_CIRCUMFERENCE = Length.fromInches(6.946136755);

  public static final double GEAR_RATIO = 4 * (50.0 / 24.0);

  public static final Length HOME_HEIGHT_FROM_FLOOR = Length.fromInches(11.713);

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getKrakenX60(4),
          GEAR_RATIO,
          Units.lbsToKilograms(20),
          PULLEY_DIAMETER.div(2).getMeters(),
          MIN_HEIGHT,
          MAX_HEIGHT.getMeters(),
          true,
          0.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(10.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elevator";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(new SimWrapper(SIM), SIM_GAINS.getPIDController());

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(8, ""), InvertedValue.Clockwise_Positive);
    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(9, ""), InvertedValue.CounterClockwise_Positive),
          new TalonFXConstants(new CANDevice(10, ""), InvertedValue.Clockwise_Positive),
          new TalonFXConstants(new CANDevice(11, ""), InvertedValue.CounterClockwise_Positive)
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = GEAR_RATIO;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 3.0 * GEAR_RATIO;
    CONFIGURATION.acceleration = CONFIGURATION.cruiseVelocity / 0.8;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(50.0, 0, 0)
            .getSlot0Configs(
                new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static));
    CONFIGURATION.positionThreshold = 0.5;
  }
}
