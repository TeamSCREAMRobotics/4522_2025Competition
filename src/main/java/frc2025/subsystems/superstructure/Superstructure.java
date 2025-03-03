package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Getter;

public class Superstructure extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private Wrist wrist;

  private final String logPrefix = "Superstructure/";

  @Getter private static SuperstructureState currentState = SuperstructureState.HOME;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
  }

  public void logTelemetry() {
    Logger.log(logPrefix + "CurrentState", getCurrentState());
  }

  public Command rezero() {
    return Commands.defer(() -> elevator.rezero(), Set.of(this, elevator, wrist));
  }

  public Command applyTargetState(SuperstructureState state) {
    Supplier<Command> command =
        () -> {
          Command transition;
          switch (state) {
            case HOME:
            case FEEDING:
              transition =
                  new ParallelCommandGroup(
                      wrist.applyGoalCommand(WristGoal.STOW),
                      new SequentialCommandGroup(
                          new WaitUntilCommand(() -> wrist.atGoal()),
                          elevator.applyGoalCommand(state.elevatorGoal)));
              break;
            default:
              transition =
                  new ParallelCommandGroup(
                      elevator.applyGoalCommand(state.elevatorGoal),
                      new SequentialCommandGroup(
                          new WaitUntilCommand(() -> elevator.atGoal()),
                          wrist.applyGoalCommand(state.wristGoal)));
              break;
          }
          transition.addRequirements(this, elevator, wrist);
          return transition;
        };
    return command.get().beforeStarting(() -> currentState = state).withName("ApplyTargetState");
  }
}
