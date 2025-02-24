package frc2025.subsystems.superstructure.elevator;

import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, ElevatorGoal.HOME);

    resetPosition(0.0);
    master.getClosedLoopReferenceSlope().setUpdateFrequency(100.0);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(Length.kZero),
    TROUGH(Length.fromInches(10.5)),
    L2(Length.fromInches(18.95)),
    L3(Length.fromInches(35.0)),
    L4(Length.fromInches(59.85)),
    CLEAR_ALGAE_L1(Length.fromInches(9.1)),
    CLEAR_ALGAE_L2(Length.fromInches(24.611)),
    BARGE(ElevatorConstants.MAX_HEIGHT.minus(Length.fromInches(8.0))),
    MAX(ElevatorConstants.MAX_HEIGHT);

    public DoubleSupplier targetRotations;

    private ElevatorGoal(Length targetHeight) {
      this.targetRotations =
          () ->
              Conversions.linearDistanceToRotations(
                  targetHeight, ElevatorConstants.PULLEY_CIRCUMFERENCE);
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return targetRotations;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  @Override
  public synchronized Command applyGoalCommand(TalonFXSubsystemGoal goal) {
    return super.applyGoalCommand(goal).beforeStarting(() -> super.goal = goal);
  }

  public Command applyUntilAtGoalCommand(ElevatorGoal goal) {
    return super.applyGoalCommand(goal)
        .until(() -> atGoal())
        .beforeStarting(() -> super.goal = goal);
  }

  public Length getMeasuredHeight() {
    return Length.fromRotations(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public Length getSetpointHeight() {
    return Length.fromRotations(getSetpoint(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  private double rotationsToHeightInches(double rotations) {
    return rotations * (2.256 * Math.PI);
  }

  public static double heightToRotations(Length height) {
    return Conversions.linearDistanceToRotations(height, ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public void resetSimController() {
    simController.reset(getPosition(), getVelocity());
  }

  public void setGoal(ElevatorGoal goal) {
    super.goal = goal;
  }

  @Override
  public void periodic() {
    super.periodic();
    /* Logger.log(logPrefix + "Height", rotationsToHeightInches(getPosition()));
    Logger.log(logPrefix + "AbsHeight", rotationsToHeightInches(getPosition()) + 10.7125);
    Logger.log(logPrefix + "ErrorHeight", rotationsToHeightInches(getError())); */
    Logger.log(logPrefix + "AppliedVolts", master.getMotorVoltage().getValueAsDouble());
    Logger.log(logPrefix + "Vel", getVelocity());
    Logger.log(logPrefix + "ReferenceVel", master.getClosedLoopReferenceSlope().getValueAsDouble());
  }
}
