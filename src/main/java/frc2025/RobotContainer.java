// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringSide;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import lombok.Getter;
import util.AllianceFlipUtil;

public class RobotContainer {

  public record Subsystems(Drivetrain drivetrain) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;

  @Getter private static final Subsystems subsystems = new Subsystems(drivetrain);

  @Getter private static final RobotState robotState = new RobotState(subsystems);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    drivetrain.registerTelemetry(RobotContainer::telemeterizeDrivetrain);
  }

  private void configureBindings() {
    Controlboard.driveController
        .a()
        .and(() -> robotState.getReefZone().isPresent())
        .whileTrue(
            drivetrain.driveToPose(
                () -> {
                  return Controlboard.getScoringSide().get() == ScoringSide.LEFT
                      ? AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(robotState.getReefZone().getAsInt())
                          .getFirst()
                      : AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(robotState.getReefZone().getAsInt())
                          .getSecond();
                }));

    Controlboard.driveController
        .x()
        .whileTrue(drivetrain.driveToBargeScoringZone(Controlboard.getTranslation()));
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
