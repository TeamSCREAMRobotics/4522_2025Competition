// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotState.GamePiece;
import frc2025.commands.DriveToPose;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringSide;
import frc2025.logging.Logger;
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
import java.util.Set;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import util.AllianceFlipUtil;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Superstructure superstructure,
      WristRollers wristRollers,
      IntakeDeploy intakeDeploy,
      IntakeRollers intakeRollers) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Superstructure superstructure =
      new Superstructure(ElevatorConstants.CONFIGURATION, WristConstants.WRIST_CONFIG);
  private static final WristRollers wristRollers = new WristRollers(WristConstants.ROLLERS_CONFIG);
  private static final IntakeDeploy intakeDeploy = new IntakeDeploy(IntakeConstants.DEPLOY_CONFIG);
  private static final IntakeRollers intakeRollers =
      new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);

  @Getter
  private static final Subsystems subsystems =
      new Subsystems(drivetrain, superstructure, wristRollers, intakeDeploy, intakeRollers);

  @Getter private static final RobotState robotState = new RobotState(subsystems);

  private final Command branchAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                  drivetrain,
                  () -> {
                    return Controlboard.getScoringSide().get() == ScoringSide.LEFT
                        ? AllianceFlipUtil.get(
                                FieldConstants.BLUE_REEF_LOCATIONS,
                                FieldConstants.RED_REEF_LOCATIONS)
                            .get(robotState.getReefZone().getAsInt())
                            .getFirst()
                        : AllianceFlipUtil.get(
                                FieldConstants.BLUE_REEF_LOCATIONS,
                                FieldConstants.RED_REEF_LOCATIONS)
                            .get(robotState.getReefZone().getAsInt())
                            .getSecond();
                  },
                  Controlboard.getTranslation()),
          Set.of(drivetrain));

  private final Command algaeAlign =
      Commands.defer(
          () ->
              new DriveToPose(
                  drivetrain,
                  () -> {
                    return AllianceFlipUtil.get(
                            FieldConstants.BLUE_ALGAE_LOCATIONS, FieldConstants.RED_ALGAE_LOCATIONS)
                        .get(robotState.getReefZone().getAsInt());
                  },
                  Controlboard.getTranslation()),
          Set.of(drivetrain));

  private final Command stationAlign =
      drivetrain.applyRequest(
          () ->
              drivetrain
                  .getHelper()
                  .getFacingAngle(
                      Controlboard.getTranslation().get(), robotState.getStationAlignAngle()));

  private final Command superstructureDefault =
      Commands.defer(
          () -> {
            SuperstructureState state;
            switch (Dashboard.selectedGamePiece()) {
              case ALGAE:
                state = SuperstructureState.IDLE_ALGAE;
                break;
              case CORAL:
                state = SuperstructureState.IDLE_CORAL;
                break;
              default:
                state = SuperstructureState.IDLE_NONE;
                break;
            }
            return superstructure.applyTargetState(state);
          },
          Set.of(superstructure, superstructure.getElevator(), superstructure.getWrist()));

  private final BooleanSupplier hasCoral = () -> Dashboard.selectedGamePiece() == GamePiece.CORAL;
  private final BooleanSupplier hasAlgae = () -> Dashboard.selectedGamePiece() == GamePiece.ALGAE;

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    drivetrain.registerTelemetry(RobotContainer::telemeterizeDrivetrain);
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
                            drivetrain.getPose().getX()
                                > AllianceFlipUtil.get(
                                        FieldConstants.BLUE_BARGE_ALIGN
                                            .getTranslation()
                                            .minus(new Translation2d(1.215, 0)),
                                        FieldConstants.RED_BARGE_ALIGN
                                            .getTranslation()
                                            .plus(new Translation2d(1.215, 0)))
                                    .getX()),
                    superstructure.applyTargetState(SuperstructureState.BARGE_NET))));

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .and(hasCoral)
        .whileTrue(superstructure.applyTargetState(SuperstructureState.REEF_L4))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.goToLevel3()
        .and(hasCoral)
        .whileTrue(superstructure.applyTargetState(SuperstructureState.REEF_L3))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.goToLevel2()
        .and(hasCoral)
        .whileTrue(superstructure.applyTargetState(SuperstructureState.REEF_L2))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(branchAlign);

    Controlboard.testButton()
        .whileTrue(superstructure.applyTargetState(SuperstructureState.HANDOFF));

    Controlboard.goToAlgaeClear2()
        .whileTrue(
            superstructure
                .applyTargetState(SuperstructureState.REEF_ALGAE_L2)
                .finallyDo(
                    () ->
                        Commands.runOnce(() -> Dashboard.setGamePiece(GamePiece.ALGAE))
                            .onlyIf(wristRollers.acquiredGamePiece())
                            .schedule()))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(algaeAlign);

    Controlboard.goToAlgaeClear1()
        .whileTrue(
            superstructure
                .applyTargetState(SuperstructureState.REEF_ALGAE_L1)
                .finallyDo(
                    () ->
                        Commands.runOnce(() -> Dashboard.setGamePiece(GamePiece.ALGAE))
                            .onlyIf(wristRollers.acquiredGamePiece())
                            .schedule()))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(algaeAlign);

    // Intake controls
    Controlboard.stationIntake()
        .whileTrue(
            Commands.parallel(
                    stationAlign,
                    superstructure.applyTargetState(SuperstructureState.CORAL_STATION),
                    intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE))
                .finallyDo(
                    () ->
                        Commands.runOnce(() -> Dashboard.setGamePiece(GamePiece.ALGAE))
                            .onlyIf(wristRollers.acquiredGamePiece())
                            .schedule()));

    Controlboard.groundIntake()
        .whileTrue(
            Commands.parallel(
                intakeDeploy.applyGoalCommand(IntakeDeployGoal.DEPLOY),
                intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE)));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                Controlboard.getFieldCentric().getAsBoolean()
                    ? drivetrain
                        .getHelper()
                        .getFieldCentric(
                            Controlboard.getTranslation().get(),
                            Controlboard.getRotation().getAsDouble())
                    : drivetrain
                        .getHelper()
                        .getRobotCentric(
                            Controlboard.getTranslation()
                                .get()
                                .times(AllianceFlipUtil.getDirectionCoefficient()),
                            Controlboard.getRotation().getAsDouble())));

    superstructure.setDefaultCommand(superstructureDefault);

    /* elevator.setDefaultCommand(
        elevator.applyVoltage(
            () ->
                (Controlboard.driveController.getLeftTriggerAxis()
                        - MathUtil.applyDeadband(Controlboard.driveController.getRightY(), 0.15))
                    * 12.0));

    wrist.setDefaultCommand(
        wrist.applyVoltage(() -> Controlboard.driveController.getRightTriggerAxis() * 24.0)); */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void logTelemetry() {
    robotState.logTelemetry();
    superstructure.logTelemetry();
  }

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    Logger.log("RobotState/RobotPose", state.Pose);
    Logger.log("RobotState/Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    Logger.log("RobotState/Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }
}
