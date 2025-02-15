package frc2025.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import math.ScreamMath;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;
import util.SimUtil;

public class ClimberConstants {

  public static final double REDUCTION = 1.0;

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(
          DCMotor.getFalcon500(2),
          REDUCTION,
          ScreamMath.parallelAxisTheorem(
                  Units.KilogramSquareMeters.of(0.0534783828),
                  Units.Pounds.of(5.5251888),
                  Length.fromInches(9.401318))
              .in(Units.KilogramSquareMeters));
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(REDUCTION, 0.0, 0.0);

  static {
    CONFIGURATION.name = "Climber";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(0.5, 0.1)),
            true,
            true);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);

    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive)
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.cruiseVelocity = 1.0;
    CONFIGURATION.acceleration = 1.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(1.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
  }
}
