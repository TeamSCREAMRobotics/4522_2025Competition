package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;

public class Feed extends Command {

  private final WristRollers rollers;

  private boolean hasSeenPiece = false;

  public Feed(WristRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
    setName("Feed");
  }

  public Feed(RobotContainer container) {
    this(container.getSubsystems().wristRollers());
  }

  @Override
  public void initialize() {
    hasSeenPiece = false;
  }

  @Override
  public void execute() {
    rollers.applyGoal(WristRollersGoal.INTAKE);
    if (rollers.hasCoral().getAsBoolean()) {
      hasSeenPiece = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      WristRollers.hasCoral = true;
    }
  }

  @Override
  public boolean isFinished() {
    return (!rollers.acquiredCoral() && hasSeenPiece) || Robot.isSimulation();
  }

  /* @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return rollers.acquiredCoral()
        ? InterruptionBehavior.kCancelIncoming
        : InterruptionBehavior.kCancelSelf;
  } */
}
