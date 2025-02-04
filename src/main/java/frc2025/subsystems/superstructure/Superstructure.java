package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.Robot;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.Direction;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import java.util.Set;
import lombok.Getter;

public class Superstructure extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private Wrist wrist;

  private final String logPrefix = "RobotState/Superstructure/";

  public SuperstructureState currentState = SuperstructureState.IDLE_NONE;
  public SuperstructureState targetState = currentState;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);
  }

  public Command applyTargetState(SuperstructureState targetState) {
    return Commands.defer(
        () -> {
          if (Robot.isSimulation()) {
            wrist.resetSimController();
            elevator.resetSimController();
          }
          this.targetState = targetState;
          Command transitionCommand = Commands.none();
          switch (targetState) {
            case HOME:
              switch(currentState){
                case BARGE_NET:
                case CLIMB:
                case HOME:
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                  transitionCommand = Commands.parallel(wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90, Direction.CLOSEST), elevator.applyUntilAtGoalCommand(ElevatorGoal.HOME));
                  break;
                case CORAL_STATION:
                case HANDOFF:
                case IDLE_ALGAE:
                case IDLE_CORAL:
                case IDLE_NONE:
                case PROCESSOR:
                case REEF_ALGAE_L1:
                case REEF_ALGAE_L2:
                default:
                  transitionCommand = Commands.sequence(wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90, Direction.CLOSEST), elevator.applyUntilAtGoalCommand(ElevatorGoal.HOME));
                  break;
              }
              break;
            case IDLE_NONE:
              switch (currentState) {
                case REEF_ALGAE_L1:
                case REEF_ALGAE_L2:
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                  transitionCommand =
                      Commands.sequence(
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.IDLE_NONE, Direction.COUNTER_CLOCKWISE),
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE));
                  break;
                case BARGE_NET:
                case CLIMB:
                case HOME:
                case IDLE_ALGAE:
                case IDLE_CORAL:
                case PROCESSOR:
                  transitionCommand = Commands.sequence(
                    wrist.applyUntilAtGoalCommand(WristGoal.STOW_CCW90, Direction.CLOSEST), elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE), wrist.applyUntilAtGoalCommand(WristGoal.IDLE_NONE, Direction.CLOSEST)
                  );
                  break;
                case CORAL_STATION:
                case HANDOFF:
                case IDLE_NONE:
                  transitionCommand = Commands.parallel(elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE), wrist.applyUntilAtGoalCommand(WristGoal.IDLE_NONE, Direction.CLOSEST));
                  break;
              }
              break;
            case IDLE_CORAL:
              switch (currentState) {
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                case CORAL_STATION:
                case BARGE_NET:
                  transitionCommand =
                      Commands.sequence(
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.IDLE_CORAL, Direction.COUNTER_CLOCKWISE),
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE_CORAL));
                  break;
                case CLIMB:
                  break;
                case HANDOFF:
                  break;
                case HOME:
                  break;
                case IDLE_ALGAE:
                  break;
                case IDLE_CORAL:
                  break;
                case IDLE_NONE:
                  break;
                case PROCESSOR:
                  break;
                case REEF_ALGAE_L1:
                  break;
                case REEF_ALGAE_L2:
                  break;
                default:
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
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.STOW_CCW90, Direction.COUNTER_CLOCKWISE),
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.IDLE_ALGAE, Direction.COUNTER_CLOCKWISE),
                          elevator.applyGoalCommand(ElevatorGoal.IDLE));
                  break;
                case IDLE_ALGAE:
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
              // Done
            case REEF_ALGAE_L1:
              break;
            case REEF_ALGAE_L2:
              break;
            case REEF_L2:
              switch (currentState) {
                case IDLE_NONE:
                case BARGE_NET:
                case CORAL_STATION:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L2),
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.REEF_L1_L3, Direction.COUNTER_CLOCKWISE));
                  break;
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                case IDLE_CORAL:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L2),
                          Commands.sequence(
                              Commands.waitUntil(
                                  () -> elevator.getMeasuredHeight().getInches() > 25.0),
                              wrist.applyUntilAtGoalCommand(WristGoal.REEF_L1_L3, Direction.CLOSEST)));
                  break;
                case CLIMB:
                  transitionCommand =
                      Commands.sequence(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.IDLE_CORAL),
                          wrist.applyUntilAtGoalCommand(WristGoal.IDLE_CORAL, Direction.CLOSEST),
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L2),
                          wrist.applyUntilAtGoalCommand(WristGoal.REEF_L1_L3, Direction.CLOSEST));
                  break;
                case HANDOFF:
                  break;
                case HOME:
                  break;
                case IDLE_ALGAE:
                  break;
                case PROCESSOR:
                  break;
                case REEF_ALGAE_L1:
                  break;
                case REEF_ALGAE_L2:
                  break;
                default:
                  break;
              }
              break;
            case REEF_L3:
              switch (currentState) {
                case IDLE_NONE:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L3),
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.REEF_L1_L3, Direction.COUNTER_CLOCKWISE));
                  break;
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                case IDLE_CORAL:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L3),
                          Commands.sequence(
                              Commands.waitUntil(
                                  () -> elevator.getMeasuredHeight().getInches() > 25.0),
                              wrist.applyUntilAtGoalCommand(WristGoal.REEF_L1_L3, Direction.CLOSEST)));
                  break;
              }
              break;
            case REEF_L4:
              switch (currentState) {
                case IDLE_NONE:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L4),
                          wrist.applyUntilAtGoalCommand(
                              WristGoal.REEF_L4, Direction.COUNTER_CLOCKWISE));
                  break;
                case REEF_L2:
                case REEF_L3:
                case REEF_L4:
                case IDLE_CORAL:
                  transitionCommand =
                      Commands.parallel(
                          elevator.applyUntilAtGoalCommand(ElevatorGoal.L4),
                          Commands.sequence(
                              Commands.waitUntil(
                                  () -> elevator.getMeasuredHeight().getInches() > 25.0),
                              wrist.applyUntilAtGoalCommand(WristGoal.REEF_L4, Direction.CLOSEST)));
                  break;
              }
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
                      wrist.applyUntilAtGoalCommand(WristGoal.HANDOFF, Direction.CLOCKWISE),
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
        Set.of(this, elevator, wrist));
  }

  public void logTelemetry() {
    Logger.log(logPrefix + "CurrentState", currentState);
    Logger.log(logPrefix + "TargetState", targetState);
  }
}
