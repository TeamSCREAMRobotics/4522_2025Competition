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
import java.util.Set;
import lombok.Getter;

public class Superstructure extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private Wrist wrist;

  private final String logPrefix = "Superstructure/";

  public SuperstructureState currentState = SuperstructureState.HOME;
  public SuperstructureState targetState = currentState;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
  }

  public void logTelemetry() {
    Logger.log(logPrefix + "CurrentState", currentState);
    Logger.log(logPrefix + "TargetState", targetState);
  }

  public Command applyTargetState(SuperstructureState state) {
    return Commands.defer(
        () -> {
          targetState = state;
          Command transition = Commands.none();
          switch (state) {
            case HOME:
              transition =
                  Commands.sequence(
                      wrist.applyUntilAtGoalCommand(WristGoal.STOW),
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.HOME));
              break;
            case REEF_ALGAE_L1:
              transition =
                  Commands.sequence(
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.CLEAR_ALGAE_L1),
                      wrist.applyUntilAtGoalCommand(WristGoal.CLEAR_ALGAE));
              break;
            case REEF_ALGAE_L2:
              transition =
                  Commands.sequence(
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.CLEAR_ALGAE_L2),
                      wrist.applyUntilAtGoalCommand(WristGoal.CLEAR_ALGAE));
              break;
            default:
              transition =
                  Commands.parallel(
                      elevator.applyGoalCommand(state.elevatorGoal),
                      wrist.applyGoalCommand(state.wristGoal));
              break;
          }
          return transition.andThen(
              Commands.parallel(
                  elevator.applyGoalCommand(state.elevatorGoal),
                  wrist.applyGoalCommand(state.wristGoal),
                  Commands.runOnce(() -> currentState = targetState)));
        },
        Set.of(this));
  }
}
