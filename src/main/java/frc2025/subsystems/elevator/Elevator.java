package frc2025.subsystems.elevator;

import dashboard.Ligament;
import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  private final Ligament elevatorLig =
      new Ligament()
          .withStaticAngle(Rotation2d.fromDegrees(90))
          .withDynamicLength(
              () -> getMeasuredHeight().plus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR),
              () ->
                  Length.fromRotations(
                          getGoal().target().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE)
                      .plus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR));

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, ElevatorGoal.HOME);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(Length.fromInches(0)),
    TROUGH(new Length()),
    L2(new Length()),
    L3(Length.fromInches(35.76)),
    L4(Length.fromInches(71.1)),
    CLEAR_ALGAE_L1(new Length()),
    CLEAR_ALGAE_L2(new Length()),
    CORAL_STATION(new Length()),
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

  public Ligament getLigament() {
    return elevatorLig;
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
            * ElevatorConstants.GEAR_RATIO,
        Conversions.mpsToRPS(
            velocity,
            ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(),
            ElevatorConstants.GEAR_RATIO));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Height", getMeasuredHeight().getInches());
  }
}
