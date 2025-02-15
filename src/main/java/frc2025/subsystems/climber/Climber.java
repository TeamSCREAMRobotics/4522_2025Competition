package frc2025.subsystems.climber;

import drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {

  public Climber(TalonFXSubsystemConfiguration config) {
    super(config, defaultGoal);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    HOME(0.0),
    PRE_CLIMB(0.5),
    CLIMB(0.1);

    private double targetRotations;

    private ClimberGoal(double targetRotations) {
      this.targetRotations = targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }
  }
}
