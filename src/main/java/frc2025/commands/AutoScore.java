// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2025.RobotContainer;
import frc2025.controlboard.Controlboard.ScoringLocation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends SequentialCommandGroup {

  private AutoAlign align;

  public AutoScore(
      RobotContainer container, SuperstructureState level, Supplier<ScoringLocation> location) {
    addRequirements(
        container.getSubsystems().drivetrain(),
        container.getSubsystems().superstructure().getElevator(),
        container.getSubsystems().superstructure().getWrist(),
        container.getSubsystems().wristRollers());

    align = new AutoAlign(container, location);
    addCommands(
        new InstantCommand(() -> container.getSubsystems().wristRollers().stop()),
        new ParallelDeadlineGroup(
            Commands.sequence(
                Commands.waitUntil(
                    () ->
                        (align.hasReachedGoal(0.025)
                            && container.getSubsystems().superstructure().atGoal())),
                container
                    .getSubsystems()
                    .wristRollers()
                    .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
                    .withTimeout(0.5)
                    .finallyDo(() -> WristRollers.resetBeam())),
            align,
            Commands.sequence(
                Commands.waitUntil(() -> align.hasReachedGoal(1.6)),
                container.applyTargetStateFactory.apply(level).get())));
  }

  public AutoScore(RobotContainer container, SuperstructureState level) {
    this(container, level, () -> container.getRobotState().getTargetScoringLocation());
  }
}
