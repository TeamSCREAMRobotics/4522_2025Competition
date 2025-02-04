package frc2025.subsystems.superstructure;

import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class SuperstructureConstants {

  public enum SuperstructureState {
    HOME(ElevatorGoal.HOME, WristGoal.STOW_CCW90),
    IDLE_NONE(ElevatorGoal.IDLE, WristGoal.IDLE_NONE),
    IDLE_CORAL(ElevatorGoal.IDLE_CORAL, WristGoal.IDLE_CORAL),
    IDLE_ALGAE(ElevatorGoal.IDLE, WristGoal.IDLE_ALGAE),
    REEF_L2(ElevatorGoal.L2, WristGoal.REEF_L1_L3),
    REEF_L3(ElevatorGoal.L3, WristGoal.REEF_L1_L3),
    REEF_L4(ElevatorGoal.L4, WristGoal.REEF_L4),
    REEF_ALGAE_L1(ElevatorGoal.CLEAR_ALGAE_L1, WristGoal.CLEAR_ALGAE),
    REEF_ALGAE_L2(ElevatorGoal.CLEAR_ALGAE_L2, WristGoal.CLEAR_ALGAE),
    CORAL_STATION(ElevatorGoal.CORAL_STATION, WristGoal.STATION),
    BARGE_NET(ElevatorGoal.BARGE, WristGoal.BARGE),
    HANDOFF(ElevatorGoal.HANDOFF, WristGoal.HANDOFF),
    PROCESSOR(ElevatorGoal.IDLE, WristGoal.IDLE_ALGAE),
    CLIMB(ElevatorGoal.HOME, WristGoal.IDLE_ALGAE);

    public WristGoal wristGoal;
    public ElevatorGoal elevatorGoal;

    private SuperstructureState(ElevatorGoal elevatorGoal, WristGoal wristGoal) {
      this.wristGoal = wristGoal;
      this.elevatorGoal = elevatorGoal;
    }
  }
}
