package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.Direction;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import java.util.function.Supplier;
import lombok.Getter;

public class Superstructure extends SubsystemBase {

  @Getter private Elevator elevator;
  @Getter private Wrist wrist;

  private SuperstructureState previousState = SuperstructureState.IDLE_NONE;

  private final Supplier<Command> holdIdleNone;
  private final Supplier<Command> holdIdleCoral;
  private final Supplier<Command> holdIdleAlgae;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    elevator = new Elevator(elevatorConfig);
    wrist = new Wrist(wristConfig);

    holdIdleNone =
        () ->
            Commands.parallel(
                wrist.applyGoal(WristGoal.IDLE_NONE), elevator.applyGoal(ElevatorGoal.IDLE));
    holdIdleCoral =
        () ->
            Commands.parallel(
                wrist.applyGoal(WristGoal.IDLE_CORAL), elevator.applyGoal(ElevatorGoal.IDLE_CORAL));
    holdIdleAlgae =
        () ->
            Commands.parallel(
                wrist.applyGoal(WristGoal.IDLE_ALGAE), elevator.applyGoal(ElevatorGoal.IDLE));
  }

  public Command applyTargetState(SuperstructureState state) {
    Supplier<Command> command = () -> Commands.none();
    switch (state) {
      case HOME:
        command =
            () ->
                Commands.sequence(
                    wrist.applyUntilAtGoal(WristGoal.STOW_CCW90),
                    elevator.applyGoal(ElevatorGoal.HOME));
      case IDLE_NONE:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
          case REEF_L2:
          case REEF_L3:
          case REEF_L4:
          case IDLE_NONE:
            command =
                () ->
                    Commands.sequence(
                        wrist.applyUntilAtGoal(WristGoal.IDLE_NONE),
                        elevator.applyUntilAtGoal(ElevatorGoal.IDLE));
            break;
        }
        break;
      case IDLE_CORAL:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
            command =
                () ->
                    Commands.sequence(
                        elevator.applyUntilAtGoal(ElevatorGoal.MAX_CARRIAGE),
                        wrist.applyUntilAtGoal(WristGoal.IDLE_CORAL, Direction.CLOCKWISE),
                        elevator.applyUntilAtGoal(ElevatorGoal.IDLE_CORAL));
            break;
          case REEF_L2:
          case REEF_L3:
          case REEF_L4:
          case BARGE_NET:
          case CORAL_STATION:
            command =
                () ->
                    Commands.sequence(
                        wrist.applyUntilAtGoal(WristGoal.IDLE_CORAL, Direction.COUNTER_CLOCKWISE),
                        elevator.applyUntilAtGoal(ElevatorGoal.IDLE_CORAL));
            break;
          default:
            command =
                () ->
                    Commands.sequence(
                        elevator.applyUntilAtGoal(ElevatorGoal.IDLE_CORAL),
                        wrist.applyUntilAtGoal(WristGoal.IDLE_CORAL));
            break;
        }
        break;
      case IDLE_ALGAE:
        switch (previousState) {
          case REEF_ALGAE_L1:
          case REEF_ALGAE_L2:
            command =
                () ->
                    Commands.sequence(
                        elevator.applyUntilAtGoal(ElevatorGoal.MAX_CARRIAGE),
                        wrist.applyUntilAtGoal(WristGoal.STOW_CCW90),
                        wrist.applyUntilAtGoal(WristGoal.IDLE_ALGAE),
                        elevator.applyGoal(ElevatorGoal.IDLE));
            break;
          case HANDOFF:
            command =
                () ->
                    Commands.sequence(
                        elevator.applyUntilAtGoal(ElevatorGoal.MAX_CARRIAGE),
                        wrist.applyUntilAtGoal(WristGoal.IDLE_ALGAE, Direction.COUNTER_CLOCKWISE),
                        elevator.applyGoal(ElevatorGoal.IDLE));
            break;
          case BARGE_NET:
            command =
                () ->
                    Commands.sequence(
                        wrist.applyUntilAtGoal(WristGoal.IDLE_ALGAE, Direction.COUNTER_CLOCKWISE),
                        elevator.applyUntilAtGoal(ElevatorGoal.IDLE));
            break;
          default:
            command = () -> holdIdleAlgae.get();
            break;
        }
        break;
      case REEF_ALGAE_L1:
        switch (previousState) {
          case IDLE_NONE:
            command =
                () ->
                    Commands.parallel(
                        elevator.applyGoal(ElevatorGoal.CLEAR_ALGAE_L1),
                        wrist.applyGoal(WristGoal.CLEAR_ALGAE, Direction.CLOCKWISE));
            break;
          default:
            break;
        }
        break;
      case REEF_ALGAE_L2:
        break;
      case REEF_L2:
        command =
            () ->
                Commands.parallel(
                    elevator.applyGoal(ElevatorGoal.L2),
                    wrist.applyGoal(WristGoal.REEF_L1_L3, Direction.CLOCKWISE));
        break;
      case REEF_L3:
        command =
            () ->
                Commands.parallel(
                    elevator.applyGoal(ElevatorGoal.L3),
                    wrist.applyGoal(WristGoal.REEF_L1_L3, Direction.CLOCKWISE));
        break;
      case REEF_L4:
        command =
            () ->
                Commands.parallel(
                    elevator.applyGoal(ElevatorGoal.L4),
                    wrist.applyGoal(WristGoal.REEF_L4, Direction.CLOCKWISE));
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
    final Supplier<Command> finalCommand = command;
    Supplier<Command> result =
        () ->
            finalCommand
                .get()
                .andThen(
                    Commands.parallel(
                        wrist.applyGoal(state.wristGoal), elevator.applyGoal(state.elevatorGoal)));
    return result.get();
  }
}
