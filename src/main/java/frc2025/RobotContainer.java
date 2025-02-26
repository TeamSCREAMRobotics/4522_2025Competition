// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.autonomous.AutoSelector;
import frc2025.commands.DriveToPose;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.subsystems.climber.Climber;
import frc2025.subsystems.climber.ClimberConstants;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import frc2025.subsystems.superstructure.wrist.WristConstants;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import frc2025.subsystems.vision.VisionManager;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import lombok.Getter;
import util.AllianceFlipUtil;
import vision.LimelightHelpers;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Superstructure superstructure,
      WristRollers wristRollers,
      Climber climber) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Superstructure superstructure =
      new Superstructure(ElevatorConstants.CONFIGURATION, WristConstants.WRIST_CONFIG);
  private static final WristRollers wristRollers = new WristRollers(WristConstants.ROLLERS_CONFIG);
  private static final Climber climber = new Climber(ClimberConstants.CONFIGURATION);

  private static final VisionManager visionManager = new VisionManager(drivetrain);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, superstructure, wristRollers, climber);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final AutoSelector autoSelector;

  private final Command branchAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                      subsystems,
                      () -> robotState.getTargetBranchPose(),
                      Controlboard.getTranslation(),
                      () ->
                          drivetrain
                                  .getEstimatedPose()
                                  .getTranslation()
                                  .getDistance(robotState.getTargetBranchPose().getTranslation())
                              < 1.5)
                  .repeatedly(),
          Set.of(drivetrain));

  private final Command algaeAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                  subsystems,
                  () -> robotState.getTargetAlgaePose(),
                  Controlboard.getTranslation(),
                  () ->
                      drivetrain
                              .getEstimatedPose()
                              .getTranslation()
                              .getDistance(robotState.getTargetBranchPose().getTranslation())
                          < 1.5),
          Set.of(drivetrain));

  private final Command stationAlign =
      drivetrain.applyRequest(
          () ->
              drivetrain
                  .getHelper()
                  .getFacingAngle(
                      Controlboard.getTranslation().get(), robotState.getStationAlignAngle()));

  private final Function<SuperstructureState, Supplier<Command>> applyTargetStateFactory =
      (state) -> () -> superstructure.applyTargetState(state);

  private final Command scoreFactory =
      Commands.defer(
          robotState.getScoreCommand(), robotState.getScoreCommand().get().getRequirements());

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();

    Controlboard.elevHeightSup = () -> superstructure.getElevator().getMeasuredHeight().getInches();

    autoSelector = new AutoSelector(this);
  }

  private void configureBindings() {

    Controlboard.driveController
        .back()
        .onTrue(Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlipUtil.getFwdHeading())));

    Controlboard.driveController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () ->
                    drivetrain.resetPose(
                        LimelightHelpers.getBotPose2d_wpiBlue("limelight-station"))));

    // Auto aligning controls
    Controlboard.alignToReef()
        .and(() -> robotState.getReefZone().isPresent())
        .toggleOnTrue(branchAlign);

    Controlboard.driveController
        .rightStick()
        .whileTrue(
            Commands.parallel(
                new DriveToPose(
                        subsystems,
                        () ->
                            AllianceFlipUtil.get(
                                FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
                        () -> Controlboard.getTranslation().get().getY())
                    .onlyIf(() -> !Dashboard.disableAutoFeatures.get()),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            drivetrain.getEstimatedPose().getX()
                                > AllianceFlipUtil.get(
                                        FieldConstants.BLUE_BARGE_ALIGN
                                            .getTranslation()
                                            .minus(new Translation2d(2.0, 0)),
                                        FieldConstants.RED_BARGE_ALIGN
                                            .getTranslation()
                                            .plus(new Translation2d(2.0, 0)))
                                    .getX()),
                    applyTargetStateFactory.apply(SuperstructureState.BARGE_NET).get())));

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L4).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(branchAlign);

    Controlboard.goToLevel3()
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L3).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(branchAlign);

    Controlboard.goToLevel2()
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L2).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(branchAlign);

    Controlboard.goToAlgaeClear2()
        .whileTrue(
            Commands.parallel(
                applyTargetStateFactory.apply(SuperstructureState.REEF_ALGAE_L2).get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)))
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(algaeAlign);

    Controlboard.goToAlgaeClear1()
        .whileTrue(
            Commands.parallel(
                applyTargetStateFactory.apply(SuperstructureState.REEF_ALGAE_L1).get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)))
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(algaeAlign);

    // Intake controls
    Controlboard.feed()
        .and(new Trigger(wristRollers.hasGamePiece()).negate())
        .whileTrue(
            Commands.parallel(
                wristRollers.feed(),
                stationAlign,
                applyTargetStateFactory.apply(SuperstructureState.FEEDING).get()));

    Controlboard.groundIntake()
        .whileTrue(
            Commands.parallel(
                applyTargetStateFactory.apply(SuperstructureState.INTAKE).get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)));

    Controlboard.lockToProcessor()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngle(
                            Controlboard.getTranslation().get(),
                            AllianceFlipUtil.get(Rotation2d.kCW_90deg, Rotation2d.kCCW_90deg))));

    Controlboard.score().whileTrue(scoreFactory);

    Controlboard.climb().onTrue(climber.retractServo());
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(
                () ->
                    Controlboard.getFieldCentric().getAsBoolean()
                        ? drivetrain
                            .getHelper()
                            .getHeadingCorrectedFieldCentric(
                                Controlboard.getTranslation().get(),
                                Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(AllianceFlipUtil.getDirectionCoefficient()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading())));

    superstructure.setDefaultCommand(
        applyTargetStateFactory.apply(SuperstructureState.FEEDING).get());
  }

  public void configureManualOverrides() {
    new Trigger(() -> Dashboard.manualMode.get())
        .whileTrue(
            Commands.parallel(
                    superstructure
                        .getElevator()
                        .applyVoltageCommand(() -> Dashboard.elevatorVoltage.get()),
                    superstructure
                        .getWrist()
                        .applyVoltageCommand(() -> Dashboard.wristVoltage.get()),
                    climber.applyVoltageCommand(() -> Dashboard.climberVoltage.get()),
                    wristRollers.applyVoltageCommand(() -> Dashboard.wristRollersVoltage.get()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> Dashboard.resetVoltageOverrides()).ignoringDisable(true));

    new Trigger(() -> Dashboard.resetVoltage.get())
        .onTrue(
            Commands.runOnce(
                () -> {
                  Dashboard.resetVoltageOverrides();
                  Dashboard.resetVoltage.set(false);
                }));

    new Trigger(() -> Dashboard.zeroElevator.get())
        .whileTrue(
            superstructure.getElevator().rezero().andThen(() -> Dashboard.zeroElevator.set(false)));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getAutonomousCommand();
  }

  public void logTelemetry() {
    robotState.logTelemetry();
    superstructure.logTelemetry();
  }
}
