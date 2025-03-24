package frc2025.subsystems.climber;

import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {

  public enum ServoGoal {
    RETRACT(0.0),
    EXTEND(1.0);

    public double position;

    private ServoGoal(double position) {
      this.position = position;
    }
  }

  public final Servo funnelServo = new Servo(9);
  public final Servo latchServo = new Servo(8);

  public static boolean hasClimbed = false;

  public Climber(TalonFXSubsystemConfiguration config) {
    super(config, ClimberGoal.HOLD_FUNNEL);

    funnelServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    latchServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    funnelServo.set(1);
    latchServo.set(1);

    resetPosition(0.0);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    HOME(0.0),
    HOLD_FUNNEL(-0.03),
    STOW_UNDER_FUNNEL(-0.069),
    OUT(0.345), // 0.335
    CLIMB(-0.0221); // -0.025

    private double targetRotations;

    private ClimberGoal(double targetRotations) {
      this.targetRotations = targetRotations;
    }


    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }
  }

  /* public Command outClimbSequence() {
    return Commands.defer(() ->
        applyGoalCommand(ClimberGoal.STOW_UNDER_FUNNEL).until(() -> atGoal()).alongWith(
        retractServo()).andThen(
        applyGoalCommand(ClimberGoal.OUT)), Set.of(this));
  } */

  public void setFunnelServo(ServoGoal goal) {
    setFunnelServo(goal.position);
  }

  public void setFunnelServo(double pos) {
    funnelServo.set(pos);
  }

  public Command setManualFunnelServo(DoubleSupplier pos) {
    return Commands.run(() -> setFunnelServo(pos.getAsDouble())).ignoringDisable(true);
  }

  public Command setFunnelServoCommand(ServoGoal goal) {
    return Commands.runOnce(() -> setFunnelServo(goal.position));
  }

  public void setLatchServo(ServoGoal goal) {
    setLatchServo(goal.position);
  }

  public void setLatchServo(double pos) {
    latchServo.set(pos);
  }

  public Command setManualLatchServo(DoubleSupplier pos) {
    return Commands.run(() -> setLatchServo(pos.getAsDouble())).ignoringDisable(true);
  }

  public Command setLatchServoCommand(ServoGoal goal) {
    return Commands.runOnce(() -> setLatchServo(goal.position));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "FunnelServoPos", funnelServo.getPosition());
  }
}
