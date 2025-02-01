package frc2025.subsystems.superstructure;

import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist.Direction;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class SuperstructureConstants {

  public enum SuperstructureState {
    HOME(ElevatorGoal.HOME, WristGoal.STOW_CCW90, Direction.CLOCKWISE),
    IDLE_NONE(ElevatorGoal.IDLE, WristGoal.IDLE_NONE, Direction.COUNTER_CLOCKWISE),
    IDLE_CORAL(ElevatorGoal.IDLE_CORAL, WristGoal.IDLE_CORAL, Direction.CLOCKWISE),
    IDLE_ALGAE(ElevatorGoal.IDLE, WristGoal.IDLE_ALGAE, Direction.CLOCKWISE),
    REEF_L2(ElevatorGoal.L2, WristGoal.REEF_L1_L3, Direction.CLOCKWISE),
    REEF_L3(ElevatorGoal.L3, WristGoal.REEF_L1_L3, Direction.CLOCKWISE),
    REEF_L4(ElevatorGoal.L4, WristGoal.REEF_L4, Direction.CLOCKWISE),
    REEF_ALGAE_L1(ElevatorGoal.CLEAR_ALGAE_L1, WristGoal.CLEAR_ALGAE, Direction.CLOCKWISE),
    REEF_ALGAE_L2(ElevatorGoal.CLEAR_ALGAE_L2, WristGoal.CLEAR_ALGAE, Direction.CLOCKWISE),
    CORAL_STATION(ElevatorGoal.CORAL_STATION, WristGoal.STATION, Direction.CLOCKWISE),
    BARGE_NET(ElevatorGoal.BARGE, WristGoal.BARGE, Direction.CLOCKWISE),
    HANDOFF(ElevatorGoal.HANDOFF, WristGoal.HANDOFF, Direction.CLOCKWISE),
    PROCESSOR(ElevatorGoal.IDLE, WristGoal.IDLE_ALGAE, Direction.CLOCKWISE),
    CLIMB(ElevatorGoal.HOME, WristGoal.STOW_CCW90, Direction.CLOCKWISE);

    public WristGoal wristGoal;
    public ElevatorGoal elevatorGoal;
    public Direction desiredDirection;

    private SuperstructureState(
        ElevatorGoal elevatorGoal, WristGoal wristGoal, Direction desiredDirection) {
      this.wristGoal = wristGoal;
      this.elevatorGoal = elevatorGoal;
      this.desiredDirection = desiredDirection;
    }
  }
}
