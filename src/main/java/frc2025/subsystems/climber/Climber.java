package frc2025.subsystems.climber;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {

  private final Servo servo_Funnel = new Servo(9);
  private final Servo servo_Latch = new Servo(0);

  public static boolean hasClimbed = false;

  public Climber(TalonFXSubsystemConfiguration config) {
    super(config, ClimberGoal.HOLD_FUNNEL);

    servo_Funnel.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    servo_Latch.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    resetPosition(0.0);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    HOME(0.0),
    HOLD_FUNNEL(-0.034),
    STOW_UNDER_FUNNEL(-0.069),
    OUT(0.335),
    CLIMB(-0.013);

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

  public Command outClimbSequence() {
    return new SequentialCommandGroup(
        applyGoalCommand(ClimberGoal.STOW_UNDER_FUNNEL).until(() -> atGoal()),
        retractServo(),
        applyGoalCommand(ClimberGoal.OUT).until(() -> atGoal()));
  }

  public Command retractServo() {
    return Commands.run(() -> servo_Funnel.setSpeed(-1.0))
        .until(() -> MathUtil.isNear(0.0, servo_Funnel.getPosition(), 0.05));
  }

  public Command extendServo() {
    return Commands.run(() -> servo_Funnel.setSpeed(1.0));
  }
}
