// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer.Subsystems;
import frc2025.subsystems.climber.Climber;
import frc2025.subsystems.climber.Climber.ClimberGoal;
import frc2025.subsystems.climber.Climber.ServoGoal;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbSequence extends Command {

  private Climber climber;

  public int index = -1;

  public double startTime = 0;

  public ClimbSequence(Subsystems subsystems) {
    climber = subsystems.climber();
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    index = -1;
    startTime = Timer.getFPGATimestamp();
    // climber.setFunnelServo(ServoGoal.RETRACT);
    climber.applyGoalCommand(ClimberGoal.OUT);
  }

  @Override
  public void execute() {
    switch (index) {
      case 0:
        // if (Timer.getFPGATimestamp() - startTime > 3.0) {
        //   climber.applyGoal(ClimberGoal.OUT);
        // } else {
        //   climber.applyGoal(ClimberGoal.STOW_UNDER_FUNNEL);
        // }
        // /* if (climber.atGoal(0.01) && climber.getGoal() == ClimberGoal.OUT) {
        //   climber.setLatchServo(ServoGoal.RETRACT);
        // } */
        if (climber.atGoal(0.01)) climber.setFunnelServo(ServoGoal.RETRACT);
        break;
      case 1:
        climber.applyGoal(ClimberGoal.CLIMB);
        break;
    }
  }

  public void advance() {
    index++;
    if (index > 1) {
      index = 0;
      startTime = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
