package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
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
}
