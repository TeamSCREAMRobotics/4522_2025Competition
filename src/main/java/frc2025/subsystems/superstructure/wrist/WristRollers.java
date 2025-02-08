package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import frc2025.Robot;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class WristRollers extends TalonFXSubsystem {

  public WristRollers(TalonFXSubsystemConfiguration config) {
    super(config, WristRollersGoal.IDLE);
  }

  public enum WristRollersGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(7.5, ControlType.VOLTAGE),
    EJECT(-7.5, ControlType.VOLTAGE);

    public final double voltage;
    public final ControlType controlType;

    private WristRollersGoal(double voltage, ControlType controlType) {
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

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  @Override
  public void periodic() {}

  public BooleanSupplier acquiredGamePiece() {
    return () ->
        (master.getSupplyCurrent().getValueAsDouble() > WristConstants.ACQUIRED_PIECE_THRESHOLD)
            || Robot.isSimulation();
  }
}
