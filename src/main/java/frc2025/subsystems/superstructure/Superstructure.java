package frc2025.subsystems.superstructure;

import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.commands.TimedCommand;
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
    return run(
        () -> {
          Solution solution = WristSolver.inverse(position);
          elevator.setSolverOutput(() -> Elevator.heightToRotations(solution.height()));
          wrist.setSolverOutput(() -> solution.wristAngle().getRotations());
        });
  }

  public Command interpolateToPosition(
      SuperstructurePosition position, boolean useBestOrientation) {
    return Commands.defer(
        () ->
            new TimedCommand(
                (t) -> {
                  double time = t * 1.75;
                  Translation2d forward =
                      WristSolver.forward(wrist.getAngle(), elevator.getMeasuredHeight());
                  Translation2d target = forward.interpolate(position.position, time);
                  Solution finalSolution =
                      useBestOrientation
                          ? WristSolver.inverse(
                              target, position.position, position.orientation, wrist.getAngle())
                          : WristSolver.inverse(target, position.orientation);
                  elevator.setSolverOutput(
                      () -> Elevator.heightToRotations(finalSolution.height()));
                  wrist.setSolverOutput(() -> finalSolution.wristAngle().getRotations());
                }),
        Set.of(this));
  }

  public Command interpolateToPosition(SuperstructurePosition position) {
    return interpolateToPosition(position, false);
  }
}
