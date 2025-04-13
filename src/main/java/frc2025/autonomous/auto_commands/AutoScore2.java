// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025.autonomous.auto_commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2025.Dashboard;
import frc2025.RobotContainer;
import frc2025.constants.FieldConstants.ReefLocation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore2 extends SequentialCommandGroup {

  private AutoAlign2 align;
//   private boolean end = false;

  public AutoScore2(
      RobotContainer container,
      SuperstructureState level,
      Supplier<ReefLocation> location,
      double rotationWait) {
    addRequirements(
        container.getSubsystems().drivetrain(),
        container.getSubsystems().superstructure().getElevator(),
        container.getSubsystems().superstructure().getWrist());

    align = new AutoAlign2(container, location.get(), false, rotationWait);
    addCommands(
        // new InstantCommand(() -> end = false),
        new InstantCommand(() -> container.getSubsystems().wristRollers().stop()),
        new ParallelDeadlineGroup(
            Commands.sequence(
                Commands.waitUntil(
                    () ->
                        (align.hasReachedGoal(Dashboard.autoScoreDistance.get())
                            && container.getSubsystems().superstructure().atGoal(level))),
                Commands.run(
                        () ->
                            container
                                .getSubsystems()
                                .wristRollers()
                                .applyGoal(WristRollersGoal.EJECT_CORAL))
                    .withTimeout(0.2)
                    .finallyDo(() -> WristRollers.resetBeam())),
            align,
            Commands.sequence(
                Commands.waitUntil(() -> align.hasReachedGoal(1.2) && WristRollers.hasCoral),
                // .raceWith(Commands.waitUntil(() -> align.hasReachedGoal(0.05))),
                // Commands.either(
                //     container.applyTargetStateFactory.apply(level).get(),
                //     Commands.runOnce(() -> cancel()),
                //     () -> WristRollers.hasCoral))));
                container.applyTargetStateFactory.apply(level).get())));
  }
}
