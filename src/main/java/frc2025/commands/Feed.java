// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc2025.subsystems.superstructure.wrist.WristRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Feed extends Command {

  private final WristRollers rollers;

  private boolean hasSeenPiece;

  public Feed(WristRollers rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasSeenPiece = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!WristRollers.hasGamePiece) {
      rollers.setVoltage(7.5);
      if (hasSeenPiece && !rollers.acquiredGamePiece().getAsBoolean()) {
        rollers.setVoltage(0.0);
      } else if (!hasSeenPiece && rollers.acquiredGamePiece().getAsBoolean()) {
        hasSeenPiece = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hasSeenPiece = false;
    if (!interrupted) {
      WristRollers.hasGamePiece = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // rollers.hasSeenPiece && !rollers.acquiredGamePiece().getAsBoolean();
  }
}
