package frc2025.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public final class ElevatorConstants {

  // As measured from wrist pivot axis
  public static final Length MIN_HEIGHT_FROM_FLOOR = Length.fromInches(8.5782);
  public static final Length MAX_HEIGHT_FROM_FLOOR = Length.fromInches(88.317193);

  public static final Translation2d ELEVATOR_ORIGIN =
      new Translation2d(0, MIN_HEIGHT_FROM_FLOOR.getMeters());

  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT =
      MAX_HEIGHT_FROM_FLOOR.minus(MIN_HEIGHT_FROM_FLOOR); // 79.738993

  // Theoretically MAX_HEIGHT / PULLEY_CIRCUMFERENCE, but needs to actually be measured
  public static final double ENCODER_MAX = 11.25075788;
  public static final double ENCODER_MIN = 0.0;
  public static final Length PULLEY_DIAMETER = Length.fromInches(2.256);
  public static final Length PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);

  public static final double REDUCTION = (40.0 / 14.0) * (50.0 / 40.0);

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          REDUCTION,
          Units.lbsToKilograms(21.44),
          PULLEY_DIAMETER.div(2).getMeters(),
          MIN_HEIGHT,
          MAX_HEIGHT.plus(Length.fromInches(0.875)).getMeters(),
          false,
          0.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(REDUCTION, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elevator";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = true;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            SIM_GAINS.getProfiledPIDController(new Constraints(10.0 * REDUCTION, 0.0)),
            false,
            true,
            false);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(
            new CANDevice(8, ""),
            InvertedValue.Clockwise_Positive); // Left as viewed from motor side
    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(
              new CANDevice(9, ""),
              InvertedValue.CounterClockwise_Positive), // Right as viewed from motor side
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 3.0 * REDUCTION;
    CONFIGURATION.acceleration = CONFIGURATION.cruiseVelocity * 0.8;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(50.0, 0, 0)
            .getSlot0Configs(
                new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static));
    CONFIGURATION.positionThreshold = 0.5;
  }
}
