package frc2025.subsystems.superstructure.wrist;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.logging.Logger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class Wrist extends TalonFXSubsystem {

  public enum Direction {
    CLOSEST,
    CLOCKWISE,
    COUNTER_CLOCKWISE;
  }

  private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0, 1.1589, 0, 0);

  private final ProfiledPIDController simController;
  private static DoubleSupplier simFeedforwardSup;

  private static BooleanSupplier shouldSimulate;

  private IntSupplier totalRotations = () -> (int) getPosition();

  private DoubleSupplier lastGoal = () -> WristGoal.IDLE_NONE.angle.getDegrees();
  private double targetDeg = 55.0;

  public Wrist(TalonFXSubsystemConfiguration config) {
    super(config);

    simController = super.simController;
    simFeedforwardSup = () -> 0.0; /*
            FEEDFORWARD.calculate(
                Units.rotationsToRadians(simController.getSetpoint().position),
                Units.rotationsToRadians(simController.getSetpoint().velocity)); */
    shouldSimulate = () -> shouldSimulate();
  }

  public enum WristGoal implements TalonFXSubsystemGoal {
    STOW_CCW90(Rotation2d.fromDegrees(90.0)),
    IDLE_NONE(Rotation2d.fromDegrees(55.0)),
    IDLE_CORAL(Rotation2d.fromDegrees(230.0)),
    IDLE_ALGAE(Rotation2d.fromDegrees(180.0)),
    REEF_L1_L3(Rotation2d.fromDegrees(196.9)),
    REEF_L4(Rotation2d.fromDegrees(209.2)),
    CLEAR_ALGAE(Rotation2d.fromDegrees(170)),
    BARGE(Rotation2d.kZero),
    STATION(Rotation2d.fromDegrees(34.55)),
    HANDOFF(Rotation2d.fromDegrees(315.0));

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
      return shouldSimulate.getAsBoolean() ? simFeedforwardSup : () -> 0.0;
    }
  }

  public Command applyGoalCommand(WristGoal goal, Direction direction) {
    return new ApplyWristGoal(goal, direction, this);
  }

  public Command applyUntilAtGoalCommand(WristGoal goal, Direction direction) {
    return applyGoalCommand(goal, direction).until(() -> atGoal());
  }

  public Command applyUntilAtGoalCommand(WristGoal goal, Direction direction, double absTolerance) {
    return applyGoalCommand(goal, direction).until(() -> atGoal(absTolerance));
  }

  public void resetSimController() {
    simController.reset(getPosition(), getVelocity());
  }

  @Override
  public boolean atGoal() {
    double error = inVelocityMode ? setpoint - getVelocity() : setpoint - getPosition();
    return inVelocityMode
        ? Math.abs(error) <= config.velocityThreshold
        : Math.abs(error) <= config.positionThreshold;
  }

  @Override
  public boolean atGoal(double absTolerance) {
    double error = inVelocityMode ? setpoint - getVelocity() : setpoint - getPosition();
    return inVelocityMode ? Math.abs(error) <= absTolerance : Math.abs(error) <= absTolerance;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(
        logPrefix + "SetpointDegrees",
        new double[] {
          Units.rotationsToDegrees(setpoint), Units.rotationsToDegrees(setpoint) % 360
        });
    /* Logger.log(logPrefix + "LastGoal", lastGoal.getAsDouble());
    Logger.log(logPrefix + "TotalRotations", totalRotations.getAsInt());
    Logger.log(logPrefix + "TargetDeg", targetDeg);
    Logger.log(logPrefix + "Feedforward", simFeedforwardSup.getAsDouble()); */
  }

  private class ApplyWristGoal extends Command {

    private final WristGoal goal;
    private final Direction direction;
    private final Wrist wrist;
    private final boolean closest;

    private double normalizedLast;
    private double normalizedGoal;
    private double goalDiff;

    public ApplyWristGoal(WristGoal goal, Direction direction, Wrist wrist) {
      this.goal = goal;
      this.direction = direction;
      this.wrist = wrist;
      this.closest = direction == Direction.CLOSEST;
      addRequirements(wrist);
    }

    @Override
    public void initialize() {
      wrist.goal = goal;
      normalizedLast = lastGoal.getAsDouble() % 360;
      normalizedLast = normalizedLast < 0 ? normalizedLast + 360 : normalizedLast;
      normalizedGoal = goal.angle.getDegrees();
      if (closest) {
        double CCWDiff = (normalizedGoal - normalizedLast + 360) % 360;
        double CWDiff = (normalizedLast - normalizedGoal + 360) % 360;
        goalDiff = (CCWDiff < CWDiff ? CCWDiff : -CWDiff);
      } else {
        goalDiff =
            direction == Direction.COUNTER_CLOCKWISE
                ? (normalizedGoal - normalizedLast + 360) % 360
                : -(normalizedLast - normalizedGoal + 360) % 360;
      }
    }

    @Override
    public void execute() {
      targetDeg = lastGoal.getAsDouble() + goalDiff;
      /* Logger.log(logPrefix + "GoalDiff", goalDiff);
      Logger.log(logPrefix + "NormalizedLast", normalizedLast);
      Logger.log(logPrefix + "NormalizedGoal", normalizedGoal); */
      setSetpointMotionMagicPosition(
          targetDeg / 360.0, shouldSimulate.getAsBoolean() ? simFeedforwardSup.getAsDouble() : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
      final double finalTarget = targetDeg;
      lastGoal = () -> finalTarget;
    }
  }
}
