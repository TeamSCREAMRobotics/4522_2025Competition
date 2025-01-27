package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructurePosition;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.WristSolver;
import frc2025.subsystems.superstructure.wrist.WristSolver.Solution;
import java.util.Set;

public class Superstructure extends SubsystemBase {

  public final Elevator elevator;
  public final Wrist wrist;

  public Superstructure(
      TalonFXSubsystemConfiguration elevatorConfig, TalonFXSubsystemConfiguration wristConfig) {
    this.elevator = new Elevator(elevatorConfig);
    this.wrist = new Wrist(wristConfig);
  }

  public Command applyPosition(SuperstructurePosition position) {
    return Commands.defer(
        () -> {
          Solution solution = WristSolver.solve(position);
          return Commands.parallel(
              elevator.setSolverOutput(() -> Elevator.heightToRotations(solution.height())),
              wrist.setSolverOutput(() -> solution.wristAngle().getRotations()));
        },
        Set.of(this));
  }
}
