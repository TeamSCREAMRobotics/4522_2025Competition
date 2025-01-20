package frc2025.subsystems.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.HOME);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(180)),
    REEF_L1_L3(Rotation2d.fromDegrees(-90)),
    REEF_L4(Rotation2d.kZero),
    CLEAR_ALGAE(Rotation2d.kZero),
    BARGE(Rotation2d.kZero),
    FUNNEL(Rotation2d.kZero);

    public final DoubleSupplier target;

    public final Rotation2d angle;

    private WristGoal(Rotation2d targetAngle) {
      this.angle = targetAngle;
      this.target = () -> targetAngle.getRotations();
    }

    @Override
    public DoubleSupplier target() {
      return target;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }
  }
}
