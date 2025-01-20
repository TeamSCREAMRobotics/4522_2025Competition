package frc2025.subsystems.elevator;

import data.Length;
import drivers.TalonFXSubsystem;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, ElevatorGoal.IDLE);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(Length.fromInches(0)),
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
  }

  public Length getMeasuredHeight() {
    return Length.fromRotations(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public Length getSetpointHeight() {
    return Length.fromRotations(getSetpoint(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  @Override
  public synchronized void setSimState(double position, double velocity) {
    super.setSimState(
        Conversions.linearDistanceToRotations(
                Length.fromMeters(position), ElevatorConstants.PULLEY_CIRCUMFERENCE)
            * ElevatorConstants.REDUCTION,
        Conversions.mpsToRPS(
            velocity,
            ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(),
            ElevatorConstants.REDUCTION));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Height", getMeasuredHeight().getInches());
  }
}
