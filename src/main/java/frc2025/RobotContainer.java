// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.autonomous.AutoSelector;
import frc2025.commands.AutoAlign;
import frc2025.commands.AutoScore;
import frc2025.commands.ClimbSequence;
import frc2025.commands.Feed;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.subsystems.climber.Climber;
import frc2025.subsystems.climber.Climber.ClimberGoal;
import frc2025.subsystems.climber.ClimberConstants;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.leds.LED;
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
      Climber climber,
      LED led) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Superstructure superstructure =
      new Superstructure(ElevatorConstants.CONFIGURATION, WristConstants.WRIST_CONFIG);
  private static final WristRollers wristRollers = new WristRollers(WristConstants.ROLLERS_CONFIG);
  private static final Climber climber = new Climber(ClimberConstants.CONFIGURATION);
  private static final LED led = new LED();

  @Getter private static final VisionManager visionManager = new VisionManager(drivetrain);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, superstructure, wristRollers, climber, led);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  @Getter private final AutoSelector autoSelector;

  public final Function<SuperstructureState, Supplier<Command>> applyTargetStateFactory =
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

  private final Command scoreFactory =
      Commands.defer(
          robotState.getScoreCommand(), robotState.getScoreCommand().get().getRequirements());

  private final Command driveDefault =
      drivetrain.run(
          () -> {
            if (Dashboard.fieldCentric.get()) {
              if (!Dashboard.disableAutoFeatures.get()
                  && superstructure.getElevator().getMeasuredHeight().getInches() < 15.0
                  && wristRollers.hasCoral
                  && !(Math.abs(Controlboard.getRotation().getAsDouble()) > 0.5)
                  && !Controlboard.groundIntake().getAsBoolean()
                  && robotState.getReefZone().isPresent()) {
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
                  () -> Dashboard.disableAutoFeatures.get() || robotState.getReefZone().isEmpty()),
          Set.of(
              drivetrain, superstructure.getElevator(), superstructure.getWrist(), wristRollers));

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

    if (Robot.isSimulation()) {
      drivetrain.resetPose(
          AllianceFlipUtil.get(
              Pose2d.kZero, new Pose2d(FieldConstants.FIELD_DIMENSIONS, Rotation2d.k180deg)));
    }
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
            /* Commands.parallel(
            new DriveToPose(
                    subsystems,
                    () ->
                        AllianceFlipUtil.get(
                            FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
                    () -> Controlboard.getTranslation().get().getY())
                .onlyIf(() -> !Dashboard.disableAutoFeatures.get()), */
            /* Commands.sequence(
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
                        || Dashboard.disableAutoFeatures.get()) */
            Commands.parallel(
                    drivetrain.applyRequest(
                        () ->
                            drivetrain
                                .getHelper()
                                .getFacingAngleProfiled(
                                    Controlboard.getTranslation()
                                        .get()
                                        .times(superstructure.getElevator().getDriveScalar() * 0.5),
                                    AllianceFlipUtil.get(
                                        Rotation2d.fromDegrees(25), Rotation2d.fromDegrees(-135)),
                                    DrivetrainConstants.HEADING_CONTROLLER_PROFILED)),
                    applyTargetStateFactory.apply(SuperstructureState.BARGE_NET).get())
                .beforeStarting(() -> drivetrain.resetHeadingController()))
        .onFalse(rezero().get());

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .and(() -> (WristRollers.hasCoral || Dashboard.disableCoralRequirement.get()))
        .whileTrue(
            Commands.either(
                applyTargetStateFactory.apply(SuperstructureState.REEF_L4).get(),
                new AutoScore(this, SuperstructureState.REEF_L4),
                () -> Dashboard.disableAutoFeatures.get()))
        .onFalse(rezero().get());
    /* .and(
        () ->
            (Controlboard.getTranslation().get().getNorm() < 0.25
                || Dashboard.disableAutoFeatures.get()))
    .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L4).get())
    .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
    .whileTrue(autoAlign); */

    Controlboard.goToLevel3()
        .and(() -> (WristRollers.hasCoral || Dashboard.disableCoralRequirement.get()))
        .whileTrue(
            Commands.either(
                applyTargetStateFactory.apply(SuperstructureState.REEF_L3).get(),
                new AutoScore(this, SuperstructureState.REEF_L3),
                () -> Dashboard.disableAutoFeatures.get()))
        .onFalse(rezero().get());
    /* .and(
        () ->
            (Controlboard.getTranslation().get().getNorm() < 0.25
                || Dashboard.disableAutoFeatures.get()))
    .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L3).get())
    .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
    .whileTrue(autoAlign); */

    Controlboard.goToLevel2()
        .and(() -> (WristRollers.hasCoral || Dashboard.disableCoralRequirement.get()))
        .whileTrue(
            Commands.either(
                applyTargetStateFactory.apply(SuperstructureState.REEF_L2).get(),
                new AutoScore(this, SuperstructureState.REEF_L2),
                () -> Dashboard.disableAutoFeatures.get()))
        .onFalse(rezero().get());
    /* .and(
        () ->
            (Controlboard.getTranslation().get().getNorm() < 0.25
                || Dashboard.disableAutoFeatures.get()))
    .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L2).get())
    .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
    .whileTrue(autoAlign); */

    Controlboard.goToTrough()
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.TROUGH).get())
        .and(() -> robotState.getReefZone().isPresent() && !Dashboard.disableAutoFeatures.get())
        .whileTrue(troughAlign);

    Controlboard.goToAlgaeClear()
        .and(() -> Controlboard.getTranslation().get().getNorm() < 0.5)
        .whileTrue(algaeClearFactory)
        .onFalse(Commands.runOnce(() -> algaeClearFactory.cancel()))
        .onFalse(rezero().get());

    // Intake controls
    Controlboard.feed()
        .and(
            () ->
                (!Dashboard.disableAutoFeatures.get()
                    && (!WristRollers.hasCoral || Dashboard.disableCoralRequirement.get())))
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

    new Trigger(() -> DriverStation.isTeleopEnabled()).onTrue(rezero().get());
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        driveDefault.beforeStarting(
            () -> {
              // drivetrain.resetHeadingController();
              drivetrain.getHelper().setLastAngle(drivetrain.getHeading());
            }));

    superstructure
        .getElevator()
        .setDefaultCommand(applyTargetStateFactory.apply(SuperstructureState.FEEDING).get());

    superstructure
        .getWrist()
        .setDefaultCommand(applyTargetStateFactory.apply(SuperstructureState.FEEDING).get());

    wristRollers.setDefaultCommand(new Feed(subsystems, Controlboard.feed()));

    led.setDefaultCommand(
        led.run(
                () -> {
                  if (DriverStation.isDisabled()) {
                    led.larson(
                        () ->
                            (AllianceFlipUtil.shouldFlip().getAsBoolean()
                                ? Color.kGreen
                                : Color.kBlue),
                        1.25);
                  } else {
                    led.wave(
                        Color.kBlack,
                        WristRollers.hasCoral ? new Color(0.1, 0.0, 0.0) : new Color(0.0, 1.0, 0.0),
                        0.1,
                        1.25);
                  }
                })
            .ignoringDisable(true));
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
                    climber.setRollersCommand(() -> Dashboard.climbRollersVoltage.get()))
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

    new Trigger(() -> Dashboard.disableClimber.get())
        .whileTrue(Commands.runOnce(() -> climber.emergencyStop()).ignoringDisable(true));

    new Trigger(() -> Dashboard.unjam.get())
        .whileTrue(
            Commands.defer(
                () ->
                    superstructure
                        .getElevator()
                        .applyVoltageCommand(() -> 1.25)
                        .alongWith(wristRollers.applyVoltageCommand(() -> 9.0)),
                Set.of(superstructure.getElevator(), wristRollers, superstructure.getWrist())));

    new Trigger(() -> Dashboard.submitRotationOverride.get())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drivetrain.resetRotation(
                          Rotation2d.fromDegrees(Dashboard.rotationOverride.get()));
                      Dashboard.submitRotationOverride.set(false);
                    })
                .ignoringDisable(true));

    new Trigger(() -> Dashboard.coastClimber.get())
        .onTrue(
            Commands.runOnce(() -> climber.setNeutralMode(NeutralModeValue.Coast))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(() -> climber.setNeutralMode(NeutralModeValue.Brake))
                .ignoringDisable(true));

    new Trigger(() -> Dashboard.coastElevator.get())
        .onTrue(
            Commands.runOnce(
                    () -> superstructure.getElevator().setNeutralMode(NeutralModeValue.Coast))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> superstructure.getElevator().setNeutralMode(NeutralModeValue.Brake))
                .ignoringDisable(true));
  }

  private Supplier<Command> rezero() {
    return () ->
        Commands.defer(
                () ->
                    applyTargetStateFactory
                        .apply(SuperstructureState.HOME)
                        .get()
                        .until(
                            () ->
                                superstructure.getElevator().getMeasuredHeight().getInches() < 0.25)
                        .andThen(superstructure.rezero()),
                Set.of(superstructure.getElevator(), superstructure.getWrist()))
            .withName("Rezero");
  }

  public Command getAutonomousCommand() {
    return autoSelector.getAutonomousCommand();
  }

  public void logTelemetry() {
    robotState.logTelemetry();
    superstructure.logTelemetry();
  }
}
