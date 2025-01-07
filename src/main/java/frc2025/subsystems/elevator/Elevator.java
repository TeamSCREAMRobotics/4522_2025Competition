package frc2025.subsystems.elevator;

import dashboard.Ligament;
import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  private final Ligament elevatorLig =
      new Ligament()
          .withStaticAngle(Rotation2d.fromDegrees(90))
          .withDynamicLength(
              () -> getMeasuredHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR),
              () ->
                  Length.fromRotations(
                          getGoal().target().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE)
                      .plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR));

  public Elevator(TalonFXSubsystemConfiguration config) {
    super(config, defaultGoal);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME(0.0);

    public double targetRotations;

    private ElevatorGoal(double targetRotations) {
      this.targetRotations = targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
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
}
