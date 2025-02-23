package frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer.Subsystems;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringSide;
import frc2025.logging.Logger;
import frc2025.sim.ComponentVisualizer;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.OptionalInt;
import java.util.function.Supplier;
import util.AllianceFlipUtil;

public class RobotState {
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final WristRollers wristRollers;

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
    this.elevator = subsystems.superstructure().getElevator();
    this.wrist = subsystems.superstructure().getWrist();
    this.wristRollers = subsystems.wristRollers();
  }

  public static Supplier<GamePiece> activeGamePiece = () -> Dashboard.Sim.selectedGamePiece();

  public enum GamePiece {
    NONE,
    CORAL,
    ALGAE;
  }

  public Supplier<Command> getScoreCommand() {
    return () -> {
      switch (Superstructure.getCurrentState()) {
        case BARGE_NET:
        case HOME:
          return wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE);
        case REEF_L2:
        case REEF_L3:
        case REEF_L4:
          return wristRollers
              .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
              .finallyDo(() -> WristRollers.resetBeam());
        default:
          return Commands.none();
      }
    };
  }

  public Pose2d getTargetBranchPose() {
    return Controlboard.getScoringSide().get() == ScoringSide.LEFT
        ? AllianceFlipUtil.get(
                FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
            .get(getReefZone().getAsInt())
            .getFirst()
        : AllianceFlipUtil.get(
                FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
            .get(getReefZone().getAsInt())
            .getSecond();
  }

  public Pose2d getTargetAlgaePose() {
    return AllianceFlipUtil.get(
            FieldConstants.BLUE_ALGAE_LOCATIONS, FieldConstants.RED_ALGAE_LOCATIONS)
        .get(getReefZone().getAsInt());
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getEstimatedPose().getTranslation());
  }

  public Rotation2d getStationAlignAngle() {
    Rotation2d angle;
    if (drivetrain.getEstimatedPose().getY() <= FieldConstants.FIELD_DIMENSIONS.getY() / 2.0) {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(55), Rotation2d.fromDegrees(125));
    } else {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(-55), Rotation2d.fromDegrees(-125));
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
    // visualizeComponents();
  }

  public void visualizeComponents() {
    Logger.log(
        "Components/MeasuredComponents",
        new Pose3d[] {
          ComponentVisualizer.getStage1Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getStage2Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getCarriagePose(elevator.getMeasuredHeight().getMeters()),
          ComponentVisualizer.getWristPose(
              elevator.getMeasuredHeight().getMeters(), wrist.getAngle())
          /* ComponentVisualizer.getCoralPose(
              elevator.getMeasuredHeight().getMeters()
                  - (activeGamePiece.get() == GamePiece.CORAL ? 0 : 4),
              wrist.getAngle()),
          ComponentVisualizer.getAlgaePose(
              elevator.getMeasuredHeight().getMeters()
                  - (activeGamePiece.get() == GamePiece.ALGAE ? 0 : 4),
              wrist.getAngle()) */
        });
    Logger.log(
        "Components/SetpointComponents",
        new Pose3d[] {
          ComponentVisualizer.getStage1Pose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getStage2Pose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getCarriagePose(elevator.getSetpointHeight().getMeters()),
          ComponentVisualizer.getWristPose(
              elevator.getSetpointHeight().getMeters(),
              Rotation2d.fromRotations(wrist.getSetpoint()))
        });
  }
}
