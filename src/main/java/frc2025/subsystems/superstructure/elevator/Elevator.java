package frc2025.subsystems.superstructure.elevator;

import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(Length.fromInches(0)),
    IDLE(Length.fromInches(12.01875)),
    IDLE_CORAL(Length.fromInches(20.0)),
    TROUGH(Length.fromInches(18.42)),
    L2(Length.fromInches(29.1)),
    L3(Length.fromInches(44.8)),
    L4(Length.fromInches(74.56)),
    CLEAR_ALGAE_L1(new Length()),
    CLEAR_ALGAE_L2(new Length()),
    CORAL_STATION(Length.fromInches(18.2)),
    HANDOFF(new Length(18.2)),
    BARGE(ElevatorConstants.MAX_HEIGHT),
    MAX_CARRIAGE(Length.fromInches(24.04)),
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

  public static double heightToRotations(Length height) {
    return Conversions.linearDistanceToRotations(height, ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public void setGoal(ElevatorGoal goal) {
    super.goal = goal;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Height", getMeasuredHeight().getInches());
    Logger.log(
        logPrefix + "AbsHeight",
        getMeasuredHeight().plus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR).getInches());
    Logger.log(
        logPrefix + "ErrorHeight",
        Length.fromRotations(getError(), ElevatorConstants.PULLEY_CIRCUMFERENCE).getInches());
  }
}
