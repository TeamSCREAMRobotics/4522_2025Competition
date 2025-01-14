package frc2025.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import frc2025.subsystems.intake.IntakeDeploy.IntakeDeployGoal;
import frc2025.subsystems.intake.IntakeRollers.IntakeRollersGoal;

public class IntakeConstants {

  public static final double DEPLOY_REDUCTION = 16.0;
  public static final double ROLLERS_REDUCTION = 2.0;

  public static final TalonFXSubsystemConfiguration DEPLOY_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    DEPLOY_CONFIG.name = "IntakeDeploy";

    DEPLOY_CONFIG.codeEnabled = true;
    DEPLOY_CONFIG.logTelemetry = false;

    DEPLOY_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive);

    DEPLOY_CONFIG.enableSupplyCurrentLimit = true;
    DEPLOY_CONFIG.supplyCurrentLimit = 20;
    DEPLOY_CONFIG.sensorToMechRatio = DEPLOY_REDUCTION;
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.Clockwise_Positive);

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
