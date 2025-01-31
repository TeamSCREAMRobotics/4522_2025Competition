package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public SuperstructureState currentState = SuperstructureState.IDLE_NONE;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
  }

  public Command applyTargetState(SuperstructureState targetState) {
    return Commands.defer(
        () -> {
          Command transitionCommand = Commands.none();
          switch (targetState) {
            case HOME:
              transitionCommand =
                  Commands.sequence(
                      wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90),
                      elevator.applyGoalCommand(ElevatorGoal.HOME));
              break;
            case IDLE_NONE:
              switch (currentState) {
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
              switch (currentState) {
                case REEF_ALGAE_L1:
                case REEF_ALGAE_L2:
                  transitionCommand =
                      Commands.sequence(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.MAX_CARRIAGE),
                          wrist.applyUntilAtGoalCommand(WristGoal.IDLE_CORAL),
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
                              WristGoal.IDLE_CORAL, Units.degreesToRotations(-50)),
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
              switch (currentState) {
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
                          wrist.applyUntilAtGoalCommand(WristGoal.IDLE_ALGAE),
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
                  break;
                case BARGE_NET:
                  transitionCommand =
                      Commands.sequence(
                          wrist.applyUntilAtGoalCommand(WristGoal.IDLE_ALGAE),
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
                  break;
                default:
                  break;
              }
              break;
            case REEF_ALGAE_L1:
              switch (currentState) {
                case IDLE_NONE:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyGoalCommand(ElevatorGoal.CLEAR_ALGAE_L1),
                          wrist.applyGoalCommand(WristGoal.CLEAR_ALGAE));
                  break;
                default:
                  break;
              }
              break;
            case REEF_ALGAE_L2:
              break;
            case REEF_L2:
              transitionCommand =
                  Commands.sequence(
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.L2),
                      wrist.applyUntilAtGoalCommand(WristGoal.REEF_L1_L3));
              break;
            case REEF_L3:
              transitionCommand =
                  Commands.parallel(
                      elevator.applyGoalCommand(ElevatorGoal.L3),
                      wrist.applyGoalCommand(WristGoal.REEF_L1_L3));
              break;
            case REEF_L4:
              transitionCommand =
                  Commands.parallel(
                      elevator.applyGoalCommand(ElevatorGoal.L4),
                      wrist.applyGoalCommand(WristGoal.REEF_L4));
              break;
            case BARGE_NET:
              break;
            case CLIMB:
              break;
            case CORAL_STATION:
              break;
            case HANDOFF:
              transitionCommand =
                  Commands.sequence(
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.MAX_CARRIAGE),
                      wrist.applyUntilAtGoalCommand(WristGoal.HANDOFF),
                      elevator.applyUntilAtGoalCommand(ElevatorGoal.HANDOFF));
              break;
            case PROCESSOR:
              break;
          }
          return transitionCommand.andThen(
              Commands.parallel(
                  wrist.applyGoalCommand(targetState.wristGoal),
                  elevator.applyGoalCommand(targetState.elevatorGoal),
                  Commands.runOnce(() -> currentState = targetState)));
        },
        Set.of(this));
  }
}
