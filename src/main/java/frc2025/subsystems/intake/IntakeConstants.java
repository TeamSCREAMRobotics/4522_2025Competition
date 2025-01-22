package frc2025.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc2025.subsystems.intake.IntakeDeploy.IntakeDeployGoal;
import frc2025.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;
import util.SimUtil;

public class IntakeConstants {

  public static final double DEPLOY_REDUCTION = 154.0;
  public static final double ROLLERS_REDUCTION = 2.0;

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(3), DEPLOY_REDUCTION, 0.015);
  public static final ScreamPIDConstants SIM_GAINS =
      new ScreamPIDConstants(75.0 * DEPLOY_REDUCTION, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration DEPLOY_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    DEPLOY_CONFIG.name = "IntakeDeploy";

    DEPLOY_CONFIG.codeEnabled = true;
    DEPLOY_CONFIG.logTelemetry = false;

    DEPLOY_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            SIM_GAINS.getPIDController(),
            false,
            false,
            ChassisReference.Clockwise_Positive);

    DEPLOY_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);

    DEPLOY_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive),
          new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive),
        };

    DEPLOY_CONFIG.neutralMode = NeutralModeValue.Brake;

    DEPLOY_CONFIG.sensorToMechRatio = DEPLOY_REDUCTION;

    DEPLOY_CONFIG.cruiseVelocity = 1.0;
    DEPLOY_CONFIG.acceleration = 1.0;
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
        new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }

  public enum IntakeGoal {
    IDLE(IntakeDeployGoal.IDLE, IntakeRollersGoal.IDLE),
    INTAKE(IntakeDeployGoal.DEPLOY, IntakeRollersGoal.INTAKE),
    EJECT(IntakeDeployGoal.IDLE, IntakeRollersGoal.EJECT);

    public final IntakeDeployGoal deployGoal;
    public final IntakeRollersGoal rollersGoal;

    private IntakeGoal(IntakeDeployGoal deployGoal, IntakeRollersGoal rollersGoal) {
      this.deployGoal = deployGoal;
      this.rollersGoal = rollersGoal;
    }
  }
}
