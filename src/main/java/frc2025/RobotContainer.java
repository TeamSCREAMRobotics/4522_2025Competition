// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.autonomous.AutoSelector;
import frc2025.commands.AutoAlign;
import frc2025.commands.ClimbSequence;
import frc2025.commands.DriveToPose;
import frc2025.commands.Feed;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.subsystems.climber.Climber;
import frc2025.subsystems.climber.Climber.ClimberGoal;
import frc2025.subsystems.climber.ClimberConstants;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
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

  @Getter private static final VisionManager visionManager = new VisionManager(drivetrain);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, superstructure, wristRollers, climber);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  @Getter private final AutoSelector autoSelector;

  private final Function<SuperstructureState, Supplier<Command>> applyTargetStateFactory =
      (state) -> () -> superstructure.applyTargetState(state);

  private final Command autoAlign =
      new AutoAlign(this, () -> robotState.getTargetScoringLocation());

  private final Command troughAlign =
      drivetrain
          .applyRequest(
              () ->
                  drivetrain
                      .getHelper()
                      .getFacingAngleProfiled(
                          Controlboard.getTranslation().get(),
                          getRobotState().getTargetAlgaeState().getFirst().getRotation(),
                          DrivetrainConstants.HEADING_CONTROLLER_PROFILED))
          .beforeStarting(() -> drivetrain.resetHeadingController());

  private final Command troughFeedAlign =
      drivetrain
          .applyRequest(
              () ->
                  drivetrain
                      .getHelper()
                      .getFacingAngleProfiled(
                          Controlboard.getTranslation().get(),
                          robotState.getStationAlignAngle().plus(new Rotation2d(Math.PI)),
                          DrivetrainConstants.HEADING_CONTROLLER_PROFILED))
          .beforeStarting(() -> drivetrain.resetHeadingController());

  private final Command feedAlign =
      drivetrain
          .applyRequest(
              () ->
                  drivetrain
                      .getHelper()
                      .getFacingAngleProfiled(
                          Controlboard.getTranslation().get(),
                          robotState.getStationAlignAngle(),
                          DrivetrainConstants.HEADING_CONTROLLER_PROFILED))
          .beforeStarting(() -> drivetrain.resetHeadingController());

  private final Command algaeClearFactory =
      Commands.defer(
          () ->
              Commands.either(
                  Commands.parallel(
                      applyTargetStateFactory
                          .apply(
                              robotState.mapAlgaeLevelToState(
                                  Dashboard.wantedAlgaeLevel.getSelected()))
                          .get(),
                      wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)),
                  Commands.parallel(
                      new AutoAlign(this, () -> robotState.getTargetScoringLocation()),
                      applyTargetStateFactory
                          .apply(robotState.getTargetAlgaeState().getSecond())
                          .get(),
                      wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)),
                  () -> Dashboard.disableAutoFeatures.get() || robotState.getReefZone().isEmpty())
          /*
          if (Dashboard.disableAutoFeatures.get() || robotState.getReefZone().isEmpty()) {
            return Commands.parallel(applyTargetStateFactory
                .apply(robotState.mapAlgaeLevelToState(Dashboard.wantedAlgaeLevel.getSelected()))
                .get(), wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE));
          } else {
            return Commands.parallel(
                new AutoAlign(this, () -> robotState.getTargetScoringLocation()),
                applyTargetStateFactory
                    .apply(robotState.getTargetAlgaeState().getSecond())
                    .get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE));
           */ ,
          Set.of(drivetrain, superstructure.getElevator(), superstructure.getWrist(), wristRollers));

  private final Command scoreFactory =
      Commands.defer(
          robotState.getScoreCommand(), robotState.getScoreCommand().get().getRequirements());

  private final Command driveDefault =
      drivetrain.run(
          () -> {
            if (Dashboard.fieldCentric.get()) {
              if (!Dashboard.disableAutoFeatures.get()
                  && superstructure.getElevator().getMeasuredHeight().getInches() < 15.0
                  && wristRollers.hasCoral().getAsBoolean()
                  && !(Math.abs(Controlboard.getRotation().getAsDouble()) > 0.5)
                  && !Controlboard.groundIntake().getAsBoolean()) {
                drivetrain.setControl(
                    drivetrain
                        .getHelper()
                        .getPointingAtProfiled(
                            Controlboard.getTranslation().get(),
                            AllianceFlipUtil.get(
                                FieldConstants.BLUE_REEF_CENTER, FieldConstants.RED_REEF_CENTER),
                            DrivetrainConstants.HEADING_CONTROLLER_PROFILED));
              } else {
                drivetrain.setControl(
                    drivetrain
                        .getHelper()
                        .getFieldCentric(
                            Controlboard.getTranslation().get(),
                            Controlboard.getRotation().getAsDouble()));
                drivetrain.resetHeadingController();
              }
            } else {
              drivetrain.setControl(
                  drivetrain
                      .getHelper()
                      .getRobotCentric(
                          Controlboard.getTranslation().get(),
                          Controlboard.getRotation().getAsDouble()));
              drivetrain.resetHeadingController();
            }
          });

  private ClimbSequence climbSequence = new ClimbSequence(subsystems);

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();

    Controlboard.elevHeightSup =
        () ->
            MathUtil.clamp(
                superstructure.getElevator().getMeasuredHeight().getInches(),
                0.0,
                ElevatorConstants.MAX_HEIGHT.getInches());

    autoSelector = new AutoSelector(this);
  }

  private void configureBindings() {

    Controlboard.driveController
        .back()
        .onTrue(Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlipUtil.getFwdHeading())));

    // Auto aligning controls
    Controlboard.alignToReef()
        .and(() -> robotState.getReefZone().isPresent())
        .toggleOnTrue(autoAlign);

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
                            AllianceFlipUtil.get(
                                    drivetrain.getEstimatedPose().getX()
                                        > FieldConstants.BLUE_BARGE_ALIGN
                                            .getTranslation()
                                            .minus(new Translation2d(3.0, 0))
                                            .getX(),
                                    drivetrain.getEstimatedPose().getX()
                                        < FieldConstants.RED_BARGE_ALIGN
                                            .getTranslation()
                                            .plus(new Translation2d(3.0, 0))
                                            .getX())
                                || Dashboard.disableAutoFeatures.get()),
                    applyTargetStateFactory.apply(SuperstructureState.BARGE_NET).get())));

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .and(
            () ->
                Controlboard.getTranslation().get().getNorm() < 0.25
                    && (wristRollers.hasCoral().getAsBoolean()
                        || Dashboard.disableCoralRequirement.get()))
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L4).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(autoAlign);

    Controlboard.goToLevel3()
        .and(
            () ->
                Controlboard.getTranslation().get().getNorm() < 0.25
                    && (wristRollers.hasCoral().getAsBoolean()
                        || Dashboard.disableCoralRequirement.get()))
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L3).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(autoAlign);

    Controlboard.goToLevel2()
        .and(
            () ->
                Controlboard.getTranslation().get().getNorm() < 0.25
                    && (wristRollers.hasCoral().getAsBoolean()
                        || Dashboard.disableCoralRequirement.get()))
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L2).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(autoAlign);

    Controlboard.goToTrough()
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.TROUGH).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(troughAlign);

    Controlboard.goToAlgaeClear()
        .and(() -> Controlboard.getTranslation().get().getNorm() < 0.5)
        .whileTrue(algaeClearFactory)
        .onFalse(Commands.runOnce(() -> algaeClearFactory.cancel()));

    // Intake controls
    Controlboard.feed()
        .whileTrue(
            Commands.parallel(
                new Feed(wristRollers),
                applyTargetStateFactory.apply(SuperstructureState.FEEDING).get()))
        .and(() -> !Dashboard.disableAutoFeatures.get())
        .whileTrue(feedAlign);

    Controlboard.troughFeed()
        .whileTrue(
            Commands.parallel(
                applyTargetStateFactory.apply(SuperstructureState.TROUGH_FEED).get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_TROUGH)))
        .and(() -> !Dashboard.disableAutoFeatures.get())
        .whileTrue(troughFeedAlign);

    Controlboard.groundIntake()
        .whileTrue(
            Commands.parallel(
                applyTargetStateFactory.apply(SuperstructureState.INTAKE).get(),
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)));

    Controlboard.processor()
        .whileTrue(
            Commands.parallel(
                drivetrain
                    .applyRequest(
                        () ->
                            drivetrain
                                .getHelper()
                                .getFacingAngleProfiled(
                                    Controlboard.getTranslation().get(),
                                    AllianceFlipUtil.get(
                                        Rotation2d.kCW_90deg, Rotation2d.kCCW_90deg),
                                    DrivetrainConstants.HEADING_CONTROLLER_PROFILED))
                    .beforeStarting(() -> drivetrain.resetHeadingController()),
                applyTargetStateFactory.apply(SuperstructureState.PROCESSOR).get()));

    Controlboard.score().whileTrue(scoreFactory);

    Controlboard.climb()
        .onTrue(
            Commands.runOnce(
                () -> {
                  climbSequence.schedule();
                  climbSequence.advance();
                }));
    // Controlboard.climb().whileTrue(climber.setFunnelServoCommand(ServoGoal.RETRACT)).whileFalse(climber.setFunnelServoCommand(ServoGoal.EXTEND));

    Controlboard.testButton().whileTrue(climber.applyGoalCommand(ClimberGoal.OUT));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        driveDefault.beforeStarting(
            () -> {
              // drivetrain.resetHeadingController();
              drivetrain.getHelper().setLastAngle(drivetrain.getHeading());
            }));

    superstructure.getElevator().setDefaultCommand(
        applyTargetStateFactory.apply(SuperstructureState.FEEDING).get());

        superstructure.getWrist().setDefaultCommand(
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
                        .applyVoltageCommand(
                            () ->
                                (Dashboard.wristVoltage.get()
                                    + superstructure.getWrist().getAngle().getCos())),
                    climber.applyVoltageCommand(() -> Dashboard.climberVoltage.get()),
                    wristRollers.applyVoltageCommand(() -> Dashboard.wristRollersVoltage.get()),
                    climber.setManualFunnelServo(() -> Dashboard.funnelServoPosition.get()),
                    climber.setManualLatchServo(() -> Dashboard.latchServoPosition.get()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> Dashboard.resetManualOverrides()).ignoringDisable(true));

    new Trigger(() -> Dashboard.resetVoltage.get())
        .onTrue(
            Commands.runOnce(
                () -> {
                  Dashboard.resetManualOverrides();
                  Dashboard.resetVoltage.set(false);
                }));

    new Trigger(() -> Dashboard.zeroElevator.get())
        .whileTrue(superstructure.rezero().andThen(() -> Dashboard.zeroElevator.set(false)));

    new Trigger(() -> Dashboard.zeroClimber.get())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      climber.resetPosition(0.0);
                      Dashboard.zeroClimber.set(false);
                    })
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getAutonomousCommand();
  }

  public void logTelemetry() {
    robotState.logTelemetry();
    superstructure.logTelemetry();
  }
}
