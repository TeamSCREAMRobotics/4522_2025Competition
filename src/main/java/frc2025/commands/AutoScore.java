// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2025.RobotContainer;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends SequentialCommandGroup {

  private AutoAlign align;

  public AutoScore(RobotContainer container, SuperstructureState level) {
    addRequirements(
        container.getSubsystems().drivetrain(),
        container.getSubsystems().superstructure().getElevator(),
        container.getSubsystems().superstructure().getWrist(),
        container.getSubsystems().wristRollers());

    align = new AutoAlign(container, () -> container.getRobotState().getTargetScoringLocation());
    addCommands(
        new InstantCommand(() -> container.getSubsystems().wristRollers().stop()),
        new ParallelCommandGroup(
            align,
            container.applyTargetStateFactory.apply(level).get(),
            Commands.sequence(
                Commands.waitUntil(
                    () ->
                        (align.hasReachedGoal(0.03)
                            && container.getSubsystems().superstructure().atGoal())),
                container
                    .getSubsystems()
                    .wristRollers()
                    .applyGoalCommand(WristRollersGoal.EJECT_CORAL))));
  }
}
