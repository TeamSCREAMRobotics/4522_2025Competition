package frc2025.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public class IntakeConstants {

  public static final double DEPLOY_REDUCTION = 110.0 / 12.0; // ~9.17
  public static final double ROLLERS_REDUCTION = 32.0 / 12.0; // ~2.7

  public static final Length PINION_CIRCUMFERENCE = Length.fromInches(Math.PI);

  public static final double MAX_EXTENSION_ROTATIONS = 5.76140894;
  public static final Length MAX_EXTENSION = Length.fromInches(18.1);

  public static final double HAS_ALGAE_CURRENT_THRESHOLD = 10.0;

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          DEPLOY_REDUCTION,
          0.1,
          PINION_CIRCUMFERENCE.getMeters(),
          0.0,
          MAX_EXTENSION.getMeters(),
          false,
          0);
  public static final ScreamPIDConstants SIM_GAINS =
      new ScreamPIDConstants(DEPLOY_REDUCTION, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration DEPLOY_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    DEPLOY_CONFIG.name = "IntakeDeploy";

    DEPLOY_CONFIG.codeEnabled = false;
    DEPLOY_CONFIG.logTelemetry = false;

    DEPLOY_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, PINION_CIRCUMFERENCE, DEPLOY_REDUCTION),
            DEPLOY_REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(60.0, 30.0)),
            false,
            false);

    DEPLOY_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);

    DEPLOY_CONFIG.neutralMode = NeutralModeValue.Brake;

    DEPLOY_CONFIG.sensorToMechRatio = DEPLOY_REDUCTION;

    DEPLOY_CONFIG.enableStatorCurrentLimit = true;
    DEPLOY_CONFIG.enableSupplyCurrentLimit = true;
    DEPLOY_CONFIG.statorCurrentLimit = 30;
    DEPLOY_CONFIG.supplyCurrentLimit = 30;

    DEPLOY_CONFIG.cruiseVelocity = 50.0;
    DEPLOY_CONFIG.acceleration = 30.0;
    DEPLOY_CONFIG.slot0 =
        new ScreamPIDConstants(1.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    DEPLOY_CONFIG.positionThreshold = 0.025;
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(11), InvertedValue.CounterClockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }
}
