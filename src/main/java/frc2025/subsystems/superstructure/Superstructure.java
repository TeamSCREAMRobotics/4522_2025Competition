package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.commands.TimedCommand;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import java.util.Set;

public class Superstructure extends SubsystemBase {

  public final Elevator elevator;
  public final Wrist wrist;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    this.elevator = new Elevator(elevatorConfig);
    this.wrist = new Wrist(wristConfig);
  }

  /* public enum SuperstructureState{
    

    public final Command commands;

    private SuperstructureState(Command... commands){
      this.commands = new SequentialCommandGroup(commands);
    }
  } */
}
