package frc2025.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructurePosition;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.WristSolver;
import frc2025.subsystems.superstructure.wrist.WristSolver.Solution;
import java.util.List;

public class SuperstructureTrajectory extends Command {
  private final Superstructure superstructure;

  private final List<SuperstructurePosition> positions;
  private final boolean useBestOrientation;
  private final Timer timer = new Timer();
  private int positionIndex = 0;
  private Translation2d forward = new Translation2d();
  private Translation2d target = new Translation2d();

  public SuperstructureTrajectory(
      List<SuperstructurePosition> positions,
      boolean useBestOrientation,
      Superstructure superstructure) {
    this.positions = positions;
    this.useBestOrientation = useBestOrientation;
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    /* if (forward.getDistance(target) > 0.5) {
      timer.stop();
    } else {
      timer.start();
    } */
    double time = timer.get() * 1.75;
    forward =
        WristSolver.forward(
            superstructure.wrist.getAngle(), superstructure.elevator.getMeasuredHeight());

    SuperstructurePosition currentPosition = positions.get(positionIndex);
    target = forward.interpolate(currentPosition.position, time);

    if (currentPosition.position.getDistance(forward) < 0.1
        && positionIndex != positions.size() - 1) {
      positionIndex++;
    }

    Solution finalSolution =
        WristSolver.inverse(
            target,
            currentPosition.position,
            currentPosition.orientation,
            superstructure.wrist.getAngle());

    superstructure.elevator.setSolverOutput(
        () -> Elevator.heightToRotations(finalSolution.height()));
    superstructure.wrist.setSolverOutput(
        () ->
            superstructure
                .wrist
                .getAngle()
                .interpolate(finalSolution.wristAngle(), time)
                .getRotations());
  }

  @Override
  public void end(boolean interrupted) {
    positionIndex = 0;
    timer.reset();
    forward = new Translation2d();
    target = new Translation2d();
  }

  @Override
  public boolean isFinished() {
    return positionIndex == positions.size() - 1
        && positions.get(positionIndex).position.getDistance(forward) < 0.1;
  }
}
