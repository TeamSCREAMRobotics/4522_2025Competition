package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.commands.SequentialGoalCommand;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class Superstructure extends SubsystemBase {

  private static Elevator elevator;
  private static Wrist wrist;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
  }

  @SuppressWarnings("unchecked")
  public enum SuperstructureState {
    IDLE(
        new SequentialGoalCommand(
            Pair.of(WristGoal.STOW_CCW90, wrist),
            Pair.of(ElevatorGoal.IDLE, elevator),
            Pair.of(WristGoal.IDLE_CCW55, wrist))),
    REEF_L2(
        new SequentialGoalCommand(
            Pair.of(ElevatorGoal.L2, elevator), Pair.of(WristGoal.REEF_L1_L3, wrist))),
    REEF_TO_IDLE(
        new SequentialGoalCommand(
            Pair.of(WristGoal.PRE_IDLE_CW45, wrist),
            Pair.of(WristGoal.IDLE_CCW55, wrist),
            Pair.of(ElevatorGoal.IDLE, elevator)));

    public final Command command;

    private SuperstructureState(SequentialGoalCommand command) {
      this.command = command;
    }
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Wrist getWrist() {
    return wrist;
  }
}
