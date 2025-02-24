package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  private final DigitalInput hasCoral = new DigitalInput(0);

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.STOW);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    STOW(Rotation2d.fromDegrees(90.0)),
    INTAKE(Rotation2d.fromDegrees(20.0)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(62.64)),
    HANDOFF(Rotation2d.fromDegrees(0.0));

    public final DoubleSupplier target;

    public final Rotation2d angle;

    private WristGoal(Rotation2d angle) {
      this.angle = angle;
      this.target = () -> angle.getRotations();
    }

    @Override
    public DoubleSupplier target() {
      return target;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  public Command applyUntilAtGoalCommand(WristGoal goal) {
    return super.applyGoalCommand(goal).until(() -> atGoal());
  }

  public Command applyUntilAtGoalCommand(WristGoal goal, double absTolerance) {
    return super.applyGoalCommand(goal).until(() -> atGoal(absTolerance));
  }

  public void resetSimController() {
    simController.reset(getPosition(), getVelocity());
  }

  public BooleanSupplier hasCoral() {
    return () -> hasCoral.get();
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
