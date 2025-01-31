package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public enum Direction {
    CLOSEST,
    OPPOSITE;
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
    IDLE_CORAL(Rotation2d.fromDegrees(-110.0)),
    IDLE_ALGAE(Rotation2d.fromDegrees(180.0)),
    PRE_IDLE_CW45(Rotation2d.fromDegrees(-45.0)),
    REEF_L1_L3(Rotation2d.fromDegrees(196.9)),
    REEF_L4(Rotation2d.fromDegrees(209.2)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(170)),
    BARGE(Rotation2d.kZero),
    STATION(Rotation2d.fromDegrees(34.55)),
    HANDOFF(Rotation2d.fromDegrees(-45.0));

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

  public Command applyUntilAtGoalCommand(TalonFXSubsystemGoal goal) {
    return super.applyGoalCommand(goal).until(() -> atGoal());
  }

  public Command applyUntilAtGoalCommand(TalonFXSubsystemGoal goal, double absTolerance) {
    return super.applyGoalCommand(goal).until(() -> atGoal(absTolerance));
  }
}
