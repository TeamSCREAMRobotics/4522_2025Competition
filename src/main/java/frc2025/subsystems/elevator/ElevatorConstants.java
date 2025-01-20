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

  // As measured from wrist pivot axis
  public static final Length MIN_HEIGHT_FROM_FLOOR = Length.fromInches(8.5782);
  public static final Length MAX_HEIGHT_FROM_FLOOR = Length.fromInches(91.1907);

  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT = Length.fromInches(82.6125);
  public static final double ENCODER_MAX = 11.6561948;
  public static final double ENCODER_MIN = 0.0;
  public static final Length PULLEY_DIAMETER = Length.fromInches(2.256);
  public static final Length PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);

  public static final double REDUCTION = (40.0 / 14.0) * (50.0 / 40.0);

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getKrakenX60(4),
          REDUCTION,
          Units.lbsToKilograms(20),
          PULLEY_DIAMETER.div(2).getMeters(),
          MIN_HEIGHT,
          MAX_HEIGHT.getMeters(),
          false,
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
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 3.0 * REDUCTION;
    CONFIGURATION.acceleration = CONFIGURATION.cruiseVelocity / 0.8;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(50.0, 0, 0)
            .getSlot0Configs(
                new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static));
    CONFIGURATION.positionThreshold = 0.5;
  }
}
