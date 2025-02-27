package frc2025.subsystems.superstructure.elevator;

import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, ElevatorGoal.FEED);

    resetPosition(0.0);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(Length.kZero),
    FEED(Length.fromInches(2.3)),
    TROUGH_FEED(Length.fromInches(14.5)),
    TROUGH(Length.fromInches(2.0)),
    L2(Length.fromInches(23.0)),
    L3(Length.fromInches(39.0)),
    L4(Length.fromInches(63.25)),
    CLEAR_ALGAE_L1(Length.fromInches(14.1)),
    CLEAR_ALGAE_L2(Length.fromInches(29.611)),
    BARGE(Length.fromInches(69.203)),
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

  private double startTime = 0.0;

  public Command rezero() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        new ParallelDeadlineGroup(
            new WaitUntilCommand(
                () ->
                    ((Timer.getFPGATimestamp() - startTime) > 2.0)
                        && master.getSupplyCurrent().getValueAsDouble() > 15.0),
            new RunCommand(() -> setVoltage(-2.0))),
        new InstantCommand(() -> resetPosition(0.0)));
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
    Logger.log(logPrefix + "Height", rotationsToHeightInches(getPosition()));
    Logger.log(logPrefix + "AbsHeight", rotationsToHeightInches(getPosition()) + 10.7125);
    Logger.log(logPrefix + "ErrorHeight", rotationsToHeightInches(getError()));
  }
}
