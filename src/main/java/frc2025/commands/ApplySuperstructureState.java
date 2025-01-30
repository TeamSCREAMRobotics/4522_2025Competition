package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.Direction;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class ApplySuperstructureState extends Command {

  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Wrist wrist;

  private SuperstructureState targetState;
  private SuperstructureState previousState;

  private Command transitionCommand;

  public ApplySuperstructureState(SuperstructureState state, Superstructure superstructure) {
    targetState = state;
    this.superstructure = superstructure;
    this.elevator = superstructure.getElevator();
    this.wrist = superstructure.getWrist();
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    transitionCommand = Commands.none();
    previousState = superstructure.currentState;
    switch (targetState) {
      case HOME:
        transitionCommand =
            Commands.sequence(
                wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90),
                elevator.applyGoalCommand(ElevatorGoal.HOME));
        break;
      case IDLE_NONE:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
          case REEF_L2:
          case REEF_L3:
          case REEF_L4:
          case IDLE_NONE:
            transitionCommand =
                Commands.sequence(
                    wrist.applyUntilAtGoalCommand(WristGoal.IDLE_NONE),
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
            break;
          default:
            break;
        }
        break;
      case IDLE_CORAL:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
            transitionCommand =
                Commands.sequence(
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.MAX_CARRIAGE),
                    wrist.applyUntilAtGoalCommand(WristGoal.IDLE_CORAL, Direction.CLOCKWISE),
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE_CORAL));
            break;
          case REEF_L2:
          case REEF_L3:
          case REEF_L4:
          case BARGE_NET:
          case CORAL_STATION:
            transitionCommand =
                Commands.sequence(
                    wrist.applyUntilAtGoalCommand(
                        WristGoal.IDLE_CORAL, Direction.COUNTER_CLOCKWISE),
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE_CORAL));
            break;
          default:
            transitionCommand =
                Commands.sequence(
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE_CORAL),
                    wrist.applyUntilAtGoalCommand(WristGoal.IDLE_CORAL));
            break;
        }
        break;
      case IDLE_ALGAE:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
            transitionCommand =
                Commands.sequence(
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.MAX_CARRIAGE),
                    wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90),
                    wrist.applyUntilAtGoalCommand(WristGoal.IDLE_ALGAE),
                    elevator.applyGoalCommand(ElevatorGoal.IDLE));
            break;
          case HANDOFF:
            transitionCommand =
                Commands.sequence(
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.MAX_CARRIAGE),
                    wrist.applyUntilAtGoalCommand(
                        WristGoal.IDLE_ALGAE, Direction.COUNTER_CLOCKWISE),
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
            break;
          case BARGE_NET:
            transitionCommand =
                Commands.sequence(
                    wrist.applyUntilAtGoalCommand(
                        WristGoal.IDLE_ALGAE, Direction.COUNTER_CLOCKWISE),
                    elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
            break;
          default:
            break;
        }
        break;
      case REEF_ALGAE_L1:
        switch (previousState) {
          case IDLE_NONE:
            transitionCommand =
                Commands.parallel(
                    elevator.applyGoalCommand(ElevatorGoal.CLEAR_ALGAE_L1),
                    wrist.applyGoalCommand(WristGoal.CLEAR_ALGAE, Direction.CLOCKWISE));
            break;
          default:
            break;
        }
        break;
      case REEF_ALGAE_L2:
        break;
      case REEF_L2:
        transitionCommand =
            Commands.parallel(
                elevator.applyGoalCommand(ElevatorGoal.L2),
                wrist.applyGoalCommand(WristGoal.REEF_L1_L3, Direction.CLOCKWISE));
        break;
      case REEF_L3:
        transitionCommand =
            Commands.parallel(
                elevator.applyGoalCommand(ElevatorGoal.L3),
                wrist.applyGoalCommand(WristGoal.REEF_L1_L3, Direction.CLOCKWISE));
        break;
      case REEF_L4:
        transitionCommand =
            Commands.parallel(
                elevator.applyGoalCommand(ElevatorGoal.L4),
                wrist.applyGoalCommand(WristGoal.REEF_L4, Direction.CLOCKWISE));
        break;
      case BARGE_NET:
        break;
      case CLIMB:
        break;
      case CORAL_STATION:
        break;
      case HANDOFF:
        break;
      case PROCESSOR:
        break;
    }
  }

  @Override
  public void execute() {
    transitionCommand.execute();
    previousState = targetState;
  }

  @Override
  public boolean isFinished() {
    return transitionCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    Commands.parallel(
            wrist.applyGoalCommand(targetState.wristGoal),
            elevator.applyGoalCommand(targetState.elevatorGoal))
        .schedule();
  }
}
