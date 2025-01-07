// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.commands.DriveToPose;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringSide;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.elevator.ElevatorConstants;
import frc2025.subsystems.intake.IntakeConstants;
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.wrist.Wrist;
import frc2025.subsystems.wrist.WristConstants;
import frc2025.subsystems.wrist.WristRollers;
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

  private int selectedReefZone = -1;

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    drivetrain.registerTelemetry(RobotContainer::telemeterizeDrivetrain);
  }

  private void configureBindings() {
    Controlboard.driveController
        .a()
        .and(() -> robotState.getReefZone().isPresent())
        .onTrue(Commands.runOnce(() -> selectedReefZone = robotState.getReefZone().getAsInt()))
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () -> {
                  return Controlboard.getScoringSide().get() == ScoringSide.LEFT
                      ? AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(selectedReefZone)
                          .getFirst()
                      : AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(selectedReefZone)
                          .getSecond();
                }));

    Controlboard.driveController
        .x()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () ->
                    AllianceFlipUtil.get(
                        FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
                () -> Controlboard.getTranslation().get().getY()));
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    DogLog.log("RobotState/RobotPose", state.Pose);
    DogLog.log("RobotState/Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    DogLog.log("RobotState/Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }
}
