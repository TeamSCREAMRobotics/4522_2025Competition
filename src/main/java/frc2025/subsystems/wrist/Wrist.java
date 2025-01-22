package frc2025.subsystems.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.IDLE);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(90.0)),
    IDLE(Rotation2d.fromDegrees(55.0)),
    REEF_L1_L3(Rotation2d.fromDegrees(196.9)),
    REEF_L4(Rotation2d.fromDegrees(209.2)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(170)),
    BARGE(Rotation2d.kZero),
    STATION(Rotation2d.fromDegrees(34.55));

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
