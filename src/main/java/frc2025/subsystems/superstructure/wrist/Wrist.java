package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Wrist extends TalonFXSubsystem {

  public enum Direction {
    CLOCKWISE,
    COUNTER_CLOCKWISE;
  }

  private final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.0, 0.3, 0.57, 0.01);

  private static DoubleSupplier solverOutput = () -> 0.0;

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.IDLE_NONE);

    removeDefaultCommand();

    simFeedforwardSup =
        () ->
            FEEDFORWARD.calculate(
                Units.rotationsToRadians(simController.getSetpoint().position),
                Units.rotationsToRadians(simController.getSetpoint().velocity));
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    STOW_CCW90(Rotation2d.fromDegrees(90.0)),
    IDLE_NONE(Rotation2d.fromDegrees(55.0)),
    IDLE_CORAL(Rotation2d.fromDegrees(-110.0)),
    IDLE_ALGAE(Rotation2d.fromDegrees(180.0)),
    PRE_IDLE_CW45(Rotation2d.fromDegrees(-45.0)),
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

  public Command applyGoal(WristGoal goal, Direction direction) {
    Supplier<Rotation2d> midpoint =
        () -> {
          double diff = goal.angle.getRadians() - getAngle().getRadians();
          double sign = direction == Direction.COUNTER_CLOCKWISE ? -1.0 : 1.0;
          return Rotation2d.fromRadians(diff * 0.5 * sign);
        };
    Supplier<Rotation2d> target = () -> goal.angle.minus(midpoint.get());
    TalonFXSubsystemGoal newGoal =
        new TalonFXSubsystemGoal() {

          @Override
          public ControlType controlType() {
            return ControlType.MOTION_MAGIC_POSITION;
          }

          @Override
          public DoubleSupplier target() {
            return () -> target.get().getRotations();
          }

          @Override
          public String toString() {
            return goal.toString();
          }
        };
    return super.applyGoal(newGoal);
  }

  public Command applyUntilAtGoal(TalonFXSubsystemGoal goal) {
    return super.applyGoal(goal).until(() -> atGoal());
  }

  public Command applyUntilAtGoal(WristGoal goal, Direction direction) {
    return applyGoal(goal, direction).until(() -> atGoal());
  }

  @Override
  public synchronized void setSimState(double position, double velocity) {
    super.setSimState(-position, velocity);
  }
}
