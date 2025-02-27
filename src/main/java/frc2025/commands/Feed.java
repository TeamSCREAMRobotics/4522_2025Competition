package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;

public class Feed extends Command {

  private final WristRollers rollers;

  public Feed(WristRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
    setName("Feed");
  }

  @Override
  public void execute() {
    rollers.applyGoal(WristRollersGoal.INTAKE);
  }

  @Override
  public void end(boolean interrupted) {
    rollers.applyGoal(WristRollersGoal.HOLD);
    WristRollers.hasGamePiece = true;
  }

  @Override
  public boolean isFinished() {
    return rollers.hasGamePiece().getAsBoolean();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return rollers.acquiredGamePiece()
        ? InterruptionBehavior.kCancelIncoming
        : InterruptionBehavior.kCancelSelf;
  }
}
