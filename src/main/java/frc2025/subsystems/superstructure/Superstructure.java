package frc2025.subsystems.superstructure;

import data.Length;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import lombok.Getter;

public class Superstructure {

  @Getter private Elevator elevator;
  @Getter private Wrist wrist;

  private final String logPrefix = "Superstructure/";

  @Getter private static SuperstructureState currentState = SuperstructureState.HOME;

  @Getter private List<Subsystem> subsystems;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
    subsystems = List.of(elevator, wrist);
  }

  public void logTelemetry() {
    Logger.log(logPrefix + "CurrentState", getCurrentState());
  }

  public boolean atGoal(SuperstructureState wanted) {
    return (elevator.atGoal() && wrist.atGoal())
        && (elevator.getGoal() == (TalonFXSubsystemGoal) wanted.elevatorGoal
            && wrist.getGoal() == (TalonFXSubsystemGoal) wanted.wristGoal);
  }

  public Command rezero() {
    return Commands.defer(() -> elevator.rezero(), Set.of(elevator, wrist));
  }

  public Command quickRezero() {
    return Commands.defer(() -> elevator.quickRezero(), Set.of(elevator, wrist));
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
                          new WaitUntilCommand(() -> (wrist.atGoal() && wrist.getGoal() == (TalonFXSubsystemGoal) WristGoal.STOW)),
                          elevator.applyGoalCommand(state.elevatorGoal)));
              break;
            default:
              transition =
                  new ParallelCommandGroup(
                      elevator.applyGoalCommand(state.elevatorGoal),
                      new RunCommand(
                          () -> {
                            if (elevator.atGoal(
                                Elevator.heightToRotations(Length.fromInches(6.0)))) {
                              wrist.applyGoal(state.wristGoal);
                            } else {
                              wrist.applyGoal(WristGoal.STOW);
                            }
                          }) /* ,
                             new SequentialCommandGroup(
                                 new WaitUntilCommand(() -> elevator.atGoal()),
                                 wrist.applyGoalCommand(state.wristGoal)) */);
              break;
          }
          transition.addRequirements(elevator, wrist);
          return transition;
        };
    return command.get().beforeStarting(() -> currentState = state).withName("ApplyTargetState");
  }
}
