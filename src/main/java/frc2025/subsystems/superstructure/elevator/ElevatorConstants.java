package frc2025.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public final class ElevatorConstants {

  // As measured from wrist pivot axis
  public static final Length MIN_HEIGHT_FROM_FLOOR = Length.fromInches(10.7125);
  public static final Length MAX_HEIGHT_FROM_FLOOR = Length.fromInches(89.825);

  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT =
      MAX_HEIGHT_FROM_FLOOR.minus(MIN_HEIGHT_FROM_FLOOR); // 79.1125

  public static final Length PULLEY_DIAMETER = Length.fromInches(2.256);
  public static final Length PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);

  // Theoretically MAX_HEIGHT / PULLEY_CIRCUMFERENCE, but needs to actually be measured
  public static final double ENCODER_MAX =
      MAX_HEIGHT.getInches() / PULLEY_CIRCUMFERENCE.getInches();
  public static final double ENCODER_MIN = 0.0;

  public static final double REDUCTION = (50.0 / 20.0) * (36.0 / 14.0);

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          REDUCTION,
          Units.lbsToKilograms(19.225),
          PULLEY_DIAMETER.div(2).getMeters(),
          MIN_HEIGHT,
          MAX_HEIGHT.getMeters(),
          false,
          0.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(5.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elevator";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, PULLEY_CIRCUMFERENCE, REDUCTION),
            REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(1.5, 1)),
            true,
            true);

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
    // CONFIGURATION.enableSupplyCurrentLimit = true;
    // CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 60.0; // 30.0
    CONFIGURATION.acceleration = 60.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(60.0, 0, 0) // 60.0
            .getSlot0Configs(
                new FeedforwardConstants(0, 0.0, 0.65, 0, GravityTypeValue.Elevator_Static));
    CONFIGURATION.positionThreshold = Elevator.heightToRotations(Length.fromInches(0.25)); // 4.0
  }
}
