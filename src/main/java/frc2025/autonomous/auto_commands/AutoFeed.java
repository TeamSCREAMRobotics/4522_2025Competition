package frc2025.autonomous.auto_commands;

import frc2025.RobotContainer;
import frc2025.commands.Feed;
import frc2025.subsystems.superstructure.wrist.WristRollers;

public class AutoFeed extends Feed {

  private final WristRollers rollers;

  public AutoFeed(RobotContainer container) {
    super(container.getSubsystems().wristRollers());
    rollers = container.getSubsystems().wristRollers();

    addRequirements(rollers);
  }

  @Override
  public boolean isFinished() {
    return rollers.hasCoral().getAsBoolean();
  }
}
