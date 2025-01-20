// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.commands.DriveToPose;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringSide;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.elevator.ElevatorConstants;
import frc2025.subsystems.intake.IntakeConstants;
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeDeploy.IntakeDeployGoal;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2025.subsystems.wrist.Wrist;
import frc2025.subsystems.wrist.Wrist.WristGoal;
import frc2025.subsystems.wrist.WristConstants;
import frc2025.subsystems.wrist.WristRollers;
import frc2025.subsystems.wrist.WristRollers.WristRollersGoal;
import java.util.Set;
import lombok.Getter;
import util.AllianceFlipUtil;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Elevator elevator,
      Wrist wrist,
      WristRollers wristRollers,
      IntakeDeploy intakeDeploy,
      IntakeRollers intakeRollers) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Elevator elevator = new Elevator(ElevatorConstants.CONFIGURATION);
  private static final Wrist wrist = new Wrist(WristConstants.WRIST_CONFIG);
  private static final WristRollers wristRollers = new WristRollers(WristConstants.ROLLERS_CONFIG);
  private static final IntakeDeploy intakeDeploy = new IntakeDeploy(IntakeConstants.DEPLOY_CONFIG);
  private static final IntakeRollers intakeRollers =
      new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);

  @Getter
  private static final Subsystems subsystems =
      new Subsystems(drivetrain, elevator, wrist, wristRollers, intakeDeploy, intakeRollers);

  @Getter private static final RobotState robotState = new RobotState(subsystems);

  private final Command reefAlign =
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

  private final Command stationAlign =
      drivetrain.applyRequest(
          () ->
              drivetrain
                  .getHelper()
                  .getFacingAngle(
                      Controlboard.getTranslation().get(), robotState.getStationAlignAngle()));

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    drivetrain.registerTelemetry(RobotContainer::telemeterizeDrivetrain);
  }

  private void configureBindings() {

    // Auto aligning controls
    Controlboard.driveController
        .povLeft()
        .and(() -> robotState.getReefZone().isPresent())
        .toggleOnTrue(reefAlign);

    Controlboard.driveController
        .rightStick()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () ->
                    AllianceFlipUtil.get(
                        FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
                () -> Controlboard.getTranslation().get().getY()));

    // Reef scoring/clearing controls
    Controlboard.goToLevel4()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.L4), wrist.applyGoal(WristGoal.REEF_L4)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(reefAlign);

    Controlboard.goToLevel3()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.L3), wrist.applyGoal(WristGoal.REEF_L1_L3)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(reefAlign);

    Controlboard.goToLevel2()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.L2), wrist.applyGoal(WristGoal.REEF_L1_L3)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(reefAlign)
        .onFalse(wrist.applyGoal(WristGoal.BETWEEN).withTimeout(0.5));

    Controlboard.goToTrough()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.TROUGH), wrist.applyGoal(WristGoal.REEF_L1_L3)))
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(reefAlign);

    Controlboard.goToAlgaeClear2()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.CLEAR_ALGAE_L2),
                wrist.applyGoal(WristGoal.CLEAR_ALGAE)));

    Controlboard.goToAlgaeClear1()
        .whileTrue(
            Commands.parallel(
                elevator.applyGoal(ElevatorGoal.CLEAR_ALGAE_L1),
                wrist.applyGoal(WristGoal.CLEAR_ALGAE)));

    // Intake controls
    Controlboard.stationIntake()
        .whileTrue(
            Commands.parallel(
                stationAlign,
                elevator.applyGoal(ElevatorGoal.CORAL_STATION),
                wrist.applyGoal(WristGoal.STATION),
                wristRollers.applyGoal(WristRollersGoal.INTAKE)));

    Controlboard.groundIntake()
        .whileTrue(
            Commands.parallel(
                intakeDeploy.applyGoal(IntakeDeployGoal.DEPLOY),
                intakeRollers.applyGoal(IntakeRollersGoal.INTAKE)));
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
                            Controlboard.getTranslation().get(),
                            Controlboard.getRotation().getAsDouble())));

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

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    Logger.log("RobotState/RobotPose", state.Pose);
    Logger.log("RobotState/Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    Logger.log("RobotState/Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }
}
