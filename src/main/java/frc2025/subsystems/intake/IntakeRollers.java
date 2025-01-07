package frc2025.subsystems.intake;

import drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends TalonFXSubsystem {

  public IntakeRollers(TalonFXSubsystemConfiguration config) {
    super(config, IntakeRollersGoal.IDLE);
  }

  public enum IntakeRollersGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(7.5, ControlType.VOLTAGE),
    EJECT(-7.5, ControlType.VOLTAGE);

    public final double voltage;
    public final ControlType controlType;

    private IntakeRollersGoal(double voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> voltage;
    }
  }
}
