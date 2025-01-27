package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class Wrist extends TalonFXSubsystem {

  private final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.0, 0.3, 0.57, 0.01);

  private static DoubleSupplier solverOutput = () -> 0.0;

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config, WristGoal.SOLVER);

    simFeedforwardSup =
        () ->
            FEEDFORWARD.calculate(
                Units.rotationsToRadians(simController.getSetpoint().position),
                Units.rotationsToRadians(simController.getSetpoint().velocity));
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    SOLVER(() -> solverOutput.getAsDouble());
    /* CLEAR_ALGAE(() -> Rotation2d.fromDegrees(170)),
    BARGE(() -> Rotation2d.kZero),
    STATION(() -> Rotation2d.fromDegrees(34.55)),
    INTERPOLATE(currentInterpolatedAngle); */

    public final DoubleSupplier target;

    private WristGoal(DoubleSupplier targetRotations) {
      this.target = targetRotations;
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

  public Command setSolverOutput(DoubleSupplier output) {
    return Commands.run(() -> solverOutput = output);
  }

  @Override
  public synchronized void setSimState(double position, double velocity) {
    super.setSimState(-position, velocity);
  }
}
