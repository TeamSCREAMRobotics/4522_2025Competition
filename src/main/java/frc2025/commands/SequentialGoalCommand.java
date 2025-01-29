package frc2025.commands;

import drivers.TalonFXSubsystem;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;

public class SequentialGoalCommand extends SequentialCommandGroup {

  private final List<Command> commands = new ArrayList<>();

  public SequentialGoalCommand(Pair<TalonFXSubsystemGoal, TalonFXSubsystem>... goals) {
    for (Pair<TalonFXSubsystemGoal, TalonFXSubsystem> goal : goals) {
      commands.add(
          goal.getSecond().applyGoal(goal.getFirst()).until(() -> goal.getSecond().atGoal()));
    }
    addCommands(commands.toArray(Command[]::new));
  }
}
