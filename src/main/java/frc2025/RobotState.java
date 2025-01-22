package frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.RobotContainer.Subsystems;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.logging.Logger;
import frc2025.sim.ComponentVisualizer;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.wrist.Wrist;
import frc2025.subsystems.wrist.WristRollers;
import java.util.OptionalInt;
import util.AllianceFlipUtil;

public class RobotState {
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final WristRollers wristRollers;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRollers intakeRollers;

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
    this.elevator = subsystems.elevator();
    this.wrist = subsystems.wrist();
    this.wristRollers = subsystems.wristRollers();
    this.intakeDeploy = subsystems.intakeDeploy();
    this.intakeRollers = subsystems.intakeRollers();
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getPose().getTranslation());
  }

  public Rotation2d getStationAlignAngle() {
    if (drivetrain.getPose().getY() <= FieldConstants.FIELD_DIMENSIONS.getY() / 2.0) {
      return AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(-125));
    } else {
      return AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(125));
    }
  }

  public void logTelemetry() {
    getReefZone()
        .ifPresent(
            reefZone ->
                Logger.log(
                    "Field/ScoringLocations",
                    new Pose2d[] {
                      AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(reefZone)
                          .getFirst(),
                      AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(reefZone)
                          .getSecond()
                    }));
    Logger.log("Controls/ScoringSide", Controlboard.getScoringSide().get());
    visualizeComponents();
  }

  public void visualizeComponents() {
    Logger.log(
        "Components/MeasuredComponents",
        new Pose3d[] {
          ComponentVisualizer.getIntakePose(intakeDeploy.getAngle()),
          ComponentVisualizer.getStage1Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getStage2Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getCarriagePose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getWristPose(
              elevator.getMeasuredHeight().getMeters(), wrist.getAngle().unaryMinus())
        });
    Logger.log(
        "Components/SetpointComponents",
        new Pose3d[] {
          ComponentVisualizer.getIntakePose(Rotation2d.fromRotations(intakeDeploy.getSetpoint())),
          ComponentVisualizer.getStage1Pose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getStage2Pose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getCarriagePose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getWristPose(
              elevator.getSetpointHeight().getMeters(),
              Rotation2d.fromRotations(-wrist.getSetpoint()))
        });
  }
}
