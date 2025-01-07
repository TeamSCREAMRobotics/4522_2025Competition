package frc2025.subsystems.wrist;

import drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, defaultGoal);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    HOME(0.0);

    public double targetRotations;

    private WristGoal(double targetRotations) {
      this.targetRotations = targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }
  }
}
