package frc2025.subsystems.climber;

import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {

  private final Servo servo = new Servo(9);

  public Climber(TalonFXSubsystemConfiguration config) {
    super(config, defaultGoal);

    servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    HOME(0.0),
    PRE_CLIMB(0.5),
    CLIMB(0.1);

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

  public Command retractServo() {
    return Commands.run(() -> servo.setSpeed(-1.0));
  }

  public Command extendServo() {
    return Commands.run(() -> servo.setSpeed(1.0));
  }
}
