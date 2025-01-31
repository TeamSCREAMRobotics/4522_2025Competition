package frc2025.subsystems.superstructure;

import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist.Direction;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class SuperstructureConstants {

  public enum SuperstructureState {
    HOME(ElevatorGoal.HOME, WristGoal.STOW_CCW90, Direction.CLOSEST),
    IDLE_NONE(ElevatorGoal.IDLE, WristGoal.IDLE_NONE, Direction.CLOSEST),
    IDLE_CORAL(ElevatorGoal.IDLE_CORAL, WristGoal.IDLE_CORAL, Direction.CLOSEST),
    IDLE_ALGAE(ElevatorGoal.IDLE, WristGoal.IDLE_ALGAE, Direction.CLOSEST),
    REEF_L2(ElevatorGoal.L2, WristGoal.REEF_L1_L3, Direction.CLOSEST),
    REEF_L3(ElevatorGoal.L3, WristGoal.REEF_L1_L3, Direction.CLOSEST),
    REEF_L4(ElevatorGoal.L4, WristGoal.REEF_L4, Direction.CLOSEST),
    REEF_ALGAE_L1(ElevatorGoal.CLEAR_ALGAE_L1, WristGoal.CLEAR_ALGAE, Direction.CLOSEST),
    REEF_ALGAE_L2(ElevatorGoal.CLEAR_ALGAE_L2, WristGoal.CLEAR_ALGAE, Direction.CLOSEST),
    CORAL_STATION(ElevatorGoal.CORAL_STATION, WristGoal.STATION, Direction.CLOSEST),
    BARGE_NET(ElevatorGoal.BARGE, WristGoal.BARGE, Direction.CLOSEST),
    HANDOFF(ElevatorGoal.HANDOFF, WristGoal.HANDOFF, Direction.CLOSEST),
    PROCESSOR(ElevatorGoal.HOME, WristGoal.STOW_CCW90, Direction.CLOSEST),
    CLIMB(ElevatorGoal.HOME, WristGoal.STOW_CCW90, Direction.CLOSEST);

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
