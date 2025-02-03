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
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import java.util.OptionalInt;
import java.util.function.Supplier;
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
    this.elevator = subsystems.superstructure().getElevator();
    this.wrist = subsystems.superstructure().getWrist();
    this.wristRollers = subsystems.wristRollers();
    this.intakeDeploy = subsystems.intakeDeploy();
    this.intakeRollers = subsystems.intakeRollers();
  }

  public static Supplier<GamePiece> activeGamePiece = () -> Dashboard.selectedGamePiece();

  public enum GamePiece {
    NONE,
    CORAL,
    ALGAE;
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getPose().getTranslation());
  }

  public Rotation2d getStationAlignAngle() {
    Rotation2d angle;
    if (drivetrain.getPose().getY() <= FieldConstants.FIELD_DIMENSIONS.getY() / 2.0) {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(-125), Rotation2d.fromDegrees(-55));
    } else {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(125), Rotation2d.fromDegrees(55));
    }
    Logger.log("RobotState/StationAlignAngle", angle.getDegrees());
    return angle;
  }

  public void logTelemetry() {
    getReefZone()
        .ifPresent(
            reefZone -> {
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
                  });
              Logger.log("Field/ReefZone", reefZone);
            });
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
              elevator.getMeasuredHeight().getMeters(), wrist.getAngle()),
          ComponentVisualizer.getCoralPose(
              elevator.getMeasuredHeight().getMeters()
                  - (activeGamePiece.get() == GamePiece.CORAL ? 0 : 4),
              wrist.getAngle()),
          ComponentVisualizer.getAlgaePose(
              elevator.getMeasuredHeight().getMeters()
                  - (activeGamePiece.get() == GamePiece.ALGAE ? 0 : 4),
              wrist.getAngle())
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
              Rotation2d.fromRotations(wrist.getSetpoint()))
        });
  }
}
