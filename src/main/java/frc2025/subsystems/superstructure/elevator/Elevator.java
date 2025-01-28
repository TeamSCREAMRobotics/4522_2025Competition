package frc2025.subsystems.superstructure.elevator;

import data.Length;
import drivers.TalonFXSubsystem;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  private static DoubleSupplier solverOutput = () -> 0.0;

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, ElevatorGoal.SOLVER);
  }

  private enum ElevatorGoal implements TalonFXSubsystemGoal {
    SOLVER(() -> solverOutput.getAsDouble());
    /* HOME(Length.fromInches(0)),
    IDLE(Length.fromInches(12.01875)),
    TROUGH(Length.fromInches(18.42)),
    L2(Length.fromInches(29.1)),
    L3(Length.fromInches(44.8)),
    L4(Length.fromInches(74.56)),
    CLEAR_ALGAE_L1(new Length()),
    CLEAR_ALGAE_L2(new Length()),
    CORAL_STATION(Length.fromInches(18.2)),
    HANDOFF(new Length()),
    BARGE(ElevatorConstants.MAX_HEIGHT),
    MAX(ElevatorConstants.MAX_HEIGHT); */

    public final DoubleSupplier targetRotations;

    private ElevatorGoal(DoubleSupplier targetRotations) {
      this.targetRotations = targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return targetRotations;
    }
  }

  public void setSolverOutput(DoubleSupplier output) {
    solverOutput = output;
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

  @Override
  public synchronized void setSimState(double position, double velocity) {
    super.setSimState(
        heightToRotations(Length.fromMeters(position)) * ElevatorConstants.REDUCTION,
        Conversions.mpsToRPS(
            velocity,
            ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(),
            ElevatorConstants.REDUCTION));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Height", getMeasuredHeight().getInches());
    Logger.log(
        logPrefix + "AbsHeight",
        getMeasuredHeight().plus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR).getInches());
  }
}
