package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
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

  public Command applyTargetState(SuperstructureState state) {
    Supplier<Command> command =
        () -> {
          Command transition;
          switch (state) {
            case HOME:
              transition =
                  Commands.parallel(
                      wrist.applyGoalCommand(WristGoal.STOW),
                      Commands.sequence(
                          Commands.waitUntil(() -> wrist.atGoal()),
                          elevator.applyGoalCommand(ElevatorGoal.HOME)));
              break;
            default:
              transition =
                  Commands.parallel(
                      elevator.applyGoalCommand(state.elevatorGoal),
                      Commands.sequence(
                          Commands.waitUntil(() -> elevator.atGoal()),
                          wrist.applyGoalCommand(state.wristGoal)));
              break;
          }
          transition.addRequirements(this);
          return transition;
        };
    return command.get().beforeStarting(() -> currentState = state);
  }
}
