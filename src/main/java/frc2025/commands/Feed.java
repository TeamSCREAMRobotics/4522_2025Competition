package frc2025.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.Dashboard;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.RobotContainer.Subsystems;
import frc2025.subsystems.leds.LED;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BooleanSupplier;

public class Feed extends Command {

  private final WristRollers rollers;
  private final LED led;

  private boolean hasSeenPiece = false;

  private BooleanSupplier intaking;
  private boolean isAuto = false;

  public Feed(Subsystems subsystems, BooleanSupplier intaking) {
    this.rollers = subsystems.wristRollers();
    this.led = subsystems.led();
    this.intaking = intaking;
    isAuto = false;
    addRequirements(rollers);
    setName("Feed");
  }

  public Feed(RobotContainer container) {
    this(container.getSubsystems(), () -> true);
    isAuto = true;
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
        && (intaking.getAsBoolean() || isAuto)) {
      rollers.applyGoal(WristRollersGoal.INTAKE);
      if (!hasSeenPiece && rollers.hasCoral().getAsBoolean()) {
        hasSeenPiece = true;
      } else if (hasSeenPiece && !rollers.hasCoral().getAsBoolean()) {
        led.strobeCommand(Color.kDarkRed, 0.1).withTimeout(0.75).schedule();
        rollers.applyGoal(WristRollersGoal.IDLE);
        WristRollers.hasCoral = true;
      }
    } else {
      rollers.applyGoal(WristRollersGoal.IDLE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (Robot.isSimulation()) {
      WristRollers.hasCoral = true;
    }
    if(isAuto){
      rollers.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return (WristRollers.hasCoral && isAuto) || (Robot.isSimulation() && isAuto);
    // return (!rollers.acquiredCoral() && hasSeenPiece) || Robot.isSimulation();
  }

  /* @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return rollers.acquiredCoral()
        ? InterruptionBehavior.kCancelIncoming
        : InterruptionBehavior.kCancelSelf;
  } */
}
