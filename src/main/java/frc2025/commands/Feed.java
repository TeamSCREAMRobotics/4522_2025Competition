package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc2025.Dashboard;
import frc2025.RobotContainer;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BooleanSupplier;

public class Feed extends Command {

  private final WristRollers rollers;

  private boolean hasSeenPiece = false;

  private BooleanSupplier intaking;

  public Feed(WristRollers rollers, BooleanSupplier intaking) {
    this.rollers = rollers;
    this.intaking = intaking;
    addRequirements(rollers);
    setName("Feed");
  }

  public Feed(RobotContainer container) {
    this(container.getSubsystems().wristRollers(), () -> true);
  }

  @Override
  public void initialize() {
    if (!WristRollers.hasCoral || Dashboard.disableCoralRequirement.get()) {
      hasSeenPiece = false;
    }
  }

  @Override
  public void execute() {
    if ((!WristRollers.hasCoral || Dashboard.disableCoralRequirement.get())
        && intaking.getAsBoolean()) {
      rollers.applyGoal(WristRollersGoal.INTAKE);
      if (!hasSeenPiece && rollers.hasCoral().getAsBoolean()) {
        hasSeenPiece = true;
      } else if (hasSeenPiece && !rollers.hasCoral().getAsBoolean()) {
        rollers.applyGoal(WristRollersGoal.IDLE);
        WristRollers.hasCoral = true;
      }
    } else {
      rollers.applyGoal(WristRollersGoal.IDLE);
    }
  }

  /* @Override
  public void end(boolean interrupted) {
    rollers.applyGoal(WristRollersGoal.IDLE);
    if (!interrupted) {
      WristRollers.hasCoral = true;
    }
  } */

  /* @Override
  public boolean isFinished() {
    return (!rollers.acquiredCoral() && hasSeenPiece) || Robot.isSimulation();
  } */

  /* @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return rollers.acquiredCoral()
        ? InterruptionBehavior.kCancelIncoming
        : InterruptionBehavior.kCancelSelf;
  } */
}
