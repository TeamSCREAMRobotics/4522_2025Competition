package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public enum Direction {
    CLOCKWISE,
    COUNTER_CLOCKWISE;
  }

  private final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.0, 0.3, 0.57, 0.01);

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
    IDLE_CORAL(Rotation2d.fromDegrees(250.0)),
    IDLE_ALGAE(Rotation2d.fromDegrees(180.0)),
    REEF_L1_L3(Rotation2d.fromDegrees(196.9)),
    REEF_L4(Rotation2d.fromDegrees(209.2)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(170)),
    BARGE(Rotation2d.kZero),
    STATION(Rotation2d.fromDegrees(34.55)),
    HANDOFF(Rotation2d.fromDegrees(315.0));

    public final DoubleSupplier target;

    public final Rotation2d CCW_Angle;
    public final Rotation2d CW_Angle;

    private WristGoal(Rotation2d CCW_Angle) {
      this.CCW_Angle = CCW_Angle;
      this.CW_Angle = new Rotation2d(-((2 * Math.PI) - CCW_Angle.getRadians()));
      this.target = () -> CCW_Angle.getRotations();
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

  public Command applyGoalCommand(WristGoal goal, Direction direction) {
    TalonFXSubsystemGoal newGoal =
        new TalonFXSubsystemGoal() {
          @Override
          public ControlType controlType() {
            return ControlType.MOTION_MAGIC_POSITION;
          }

          @Override
          public DoubleSupplier target() {
            return () ->
                direction == Direction.COUNTER_CLOCKWISE
                    ? goal.CCW_Angle.getRotations()
                    : goal.CW_Angle.getRotations();
          }

          @Override
          public String toString() {
            return goal.toString();
          }
        };
    return super.applyGoalCommand(newGoal).beforeStarting(() -> super.goal = newGoal);
  }

  public Command applyUntilAtGoalCommand(WristGoal goal, Direction direction) {
    return applyGoalCommand(goal, direction).until(() -> atGoal());
  }

  public Command applyUntilAtGoalCommand(WristGoal goal, Direction direction, double absTolerance) {
    return applyGoalCommand(goal, direction).until(() -> atGoal(absTolerance));
  }
}
