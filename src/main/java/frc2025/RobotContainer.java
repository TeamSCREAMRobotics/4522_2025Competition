// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotState.GamePiece;
import frc2025.autonomous.AutoSelector;
import frc2025.commands.DriveToPose;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.subsystems.climber.Climber;
import frc2025.subsystems.climber.ClimberConstants;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.intake.IntakeConstants;
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeDeploy.IntakeDeployGoal;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import frc2025.subsystems.superstructure.wrist.WristConstants;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import frc2025.subsystems.vision.VisionManager;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import lombok.Getter;
import util.AllianceFlipUtil;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Superstructure superstructure,
      WristRollers wristRollers,
      IntakeDeploy intakeDeploy,
      IntakeRollers intakeRollers,
      Climber climber) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Superstructure superstructure =
      new Superstructure(ElevatorConstants.CONFIGURATION, WristConstants.WRIST_CONFIG);
  private static final WristRollers wristRollers = new WristRollers(WristConstants.ROLLERS_CONFIG);
  private static final IntakeDeploy intakeDeploy = new IntakeDeploy(IntakeConstants.DEPLOY_CONFIG);
  private static final IntakeRollers intakeRollers =
      new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private static final Climber climber = new Climber(ClimberConstants.CONFIGURATION);

  private static final VisionManager visionManager = new VisionManager(drivetrain);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(
          drivetrain, superstructure, wristRollers, intakeDeploy, intakeRollers, climber);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final AutoSelector autoSelector;

  private final Command branchAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                      drivetrain,
                      () -> robotState.getTargetBranchPose(),
                      Controlboard.getTranslation(),
                      () ->
                          drivetrain
                                  .getGlobalPoseEstimate()
                                  .getTranslation()
                                  .getDistance(robotState.getTargetBranchPose().getTranslation())
                              < 1.5)
                  .repeatedly(),
          Set.of(drivetrain));

  private final Command algaeAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                  drivetrain,
                  () -> robotState.getTargetAlgaePose(),
                  Controlboard.getTranslation(),
                  () ->
                      drivetrain
                              .getGlobalPoseEstimate()
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

  private final BooleanSupplier hasCoral =
      () -> Dashboard.Sim.selectedGamePiece() == GamePiece.CORAL;
  private final BooleanSupplier hasAlgae =
      () -> Dashboard.Sim.selectedGamePiece() == GamePiece.ALGAE;

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    autoSelector = new AutoSelector(this);
  }

  private void configureBindings() {

    // Auto aligning controls
    Controlboard.alignToReef()
        .and(() -> robotState.getReefZone().isPresent())
        .toggleOnTrue(branchAlign);

    Controlboard.driveController
        .rightStick()
        .whileTrue(
            Commands.parallel(
                new DriveToPose(
                    drivetrain,
                    () ->
                        AllianceFlipUtil.get(
                            FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
                    () -> Controlboard.getTranslation().get().getY()),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            drivetrain.getGlobalPoseEstimate().getX()
                                > AllianceFlipUtil.get(
                                        FieldConstants.BLUE_BARGE_ALIGN
                                            .getTranslation()
                                            .minus(new Translation2d(1.215, 0)),
                                        FieldConstants.RED_BARGE_ALIGN
                                            .getTranslation()
                                            .plus(new Translation2d(1.215, 0)))
                                    .getX()),
                    applyTargetStateFactory.apply(SuperstructureState.BARGE_NET).get())));

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .and(hasCoral)
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L4).get())
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.goToLevel3()
        .and(hasCoral)
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L3).get())
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.goToLevel2()
        .and(hasCoral)
        .whileTrue(applyTargetStateFactory.apply(SuperstructureState.REEF_L2).get())
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.goToAlgaeClear2()
        .whileTrue(
            applyTargetStateFactory
                .apply(SuperstructureState.REEF_ALGAE_L2)
                .get()
                .finallyDo(() -> Dashboard.Sim.setGamePiece(GamePiece.ALGAE)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(algaeAlign);

    Controlboard.goToAlgaeClear1()
        .whileTrue(
            applyTargetStateFactory
                .apply(SuperstructureState.REEF_ALGAE_L1)
                .get()
                .finallyDo(() -> Dashboard.Sim.setGamePiece(GamePiece.ALGAE)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(algaeAlign);

    // Intake controls
    Controlboard.stationIntake()
        .whileTrue(
            Commands.parallel(
                    stationAlign,
                    applyTargetStateFactory.apply(SuperstructureState.HOME).get(),
                    wristRollers.applyGoalCommand(WristRollersGoal.INTAKE))
                .finallyDo(() -> Dashboard.Sim.setGamePiece(GamePiece.CORAL)));

    Controlboard.groundIntake()
        .whileTrue(
            Commands.parallel(
                intakeDeploy.applyGoalCommand(IntakeDeployGoal.MAX),
                intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE)));

    Controlboard.lockToProcessor()
        .whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngle(
                                Controlboard.getTranslation().get(),
                                AllianceFlipUtil.get(
                                    Rotation2d.kCW_90deg, Rotation2d.kCCW_90deg)))));

    Controlboard.score().whileTrue(scoreFactory);
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

    superstructure.setDefaultCommand(applyTargetStateFactory.apply(SuperstructureState.HOME).get());

    /* superstructure
        .getElevator()
        .setDefaultCommand(
            superstructure
                .getElevator()
                .applyVoltageCommand(
                    () ->
                        (-MathUtil.applyDeadband(Controlboard.driveController.getRightY(), 0.15))
                            * 12.0));

    superstructure
        .getWrist()
        .setDefaultCommand(
            superstructure
                .getWrist()
                .applyVoltageCommand(
                    () ->
                        (-Controlboard.driveController.getRightTriggerAxis()
                                + Controlboard.driveController.getLeftTriggerAxis())
                            * 24.0)); */
  }

  public Command getAutonomousCommand() {
    return autoSelector.getAutonomousCommand();
  }

  public void logTelemetry() {
    robotState.logTelemetry();
    superstructure.logTelemetry();
  }
}
