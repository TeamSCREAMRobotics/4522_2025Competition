package frc2025.subsystems.superstructure;

import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;

public class SuperstructureConstants {

  public enum SuperstructureState {
    HOME(ElevatorGoal.HOME, WristGoal.STOW),
    TROUGH(ElevatorGoal.TROUGH, WristGoal.TROUGH),
    REEF_L2(ElevatorGoal.L2, WristGoal.STOW),
    REEF_L3(ElevatorGoal.L3, WristGoal.STOW),
    REEF_L4(ElevatorGoal.L4, WristGoal.STOW),
    REEF_ALGAE_L1(ElevatorGoal.CLEAR_ALGAE_L1, WristGoal.CLEAR_ALGAE),
    REEF_ALGAE_L2(ElevatorGoal.CLEAR_ALGAE_L2, WristGoal.CLEAR_ALGAE),
    FEEDING(ElevatorGoal.FEED, WristGoal.STOW),
    TROUGH_FEED(ElevatorGoal.TROUGH_FEED, WristGoal.TROUGH_FEED),
    BARGE_NET(ElevatorGoal.BARGE, WristGoal.STOW),
    INTAKE(ElevatorGoal.HOME, WristGoal.INTAKE),
    PROCESSOR(ElevatorGoal.HOME, WristGoal.CLEAR_ALGAE);

    public WristGoal wristGoal;
    public ElevatorGoal elevatorGoal;

    private SuperstructureState(ElevatorGoal elevatorGoal, WristGoal wristGoal) {
      this.wristGoal = wristGoal;
      this.elevatorGoal = elevatorGoal;
    }
  }
}
