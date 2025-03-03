package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.STOW);
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    STOW(Rotation2d.fromDegrees(90.0)),
    INTAKE(Rotation2d.fromDegrees(25.0)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(62.64)),
    TROUGH_FEED(Rotation2d.fromDegrees(81.0)),
    TROUGH(Rotation2d.fromDegrees(74.0));

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

  @Override
  public void periodic() {
    super.periodic();
  }
}
