package frc2025.subsystems.wrist;

import dashboard.Ligament;
import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  private final Ligament wrist =
      new Ligament()
          .withStaticLength(Length.fromInches(17.5))
          .withDynamicAngle(
              () -> getAngle(), () -> Rotation2d.fromRotations(getGoal().target().getAsDouble()));

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.HOME);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(180)),
    SCORE_L1_L3(Rotation2d.fromDegrees(-90)),
    SCORE_L4(Rotation2d.kZero),
    CORAL_STATION(Rotation2d.kZero),
    ;

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

  public Ligament getLigament() {
    return wrist;
  }
}
