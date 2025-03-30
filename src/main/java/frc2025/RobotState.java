package frc2025;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer.Subsystems;
import frc2025.constants.FieldConstants;
import frc2025.constants.FieldConstants.AlgaeLevel;
import frc2025.controlboard.Controlboard;
import frc2025.controlboard.Controlboard.ScoringLocation;
import frc2025.logging.Logger;
import frc2025.sim.ComponentVisualizer;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
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

  public enum Coral {
    NONE,
    CORAL,
    ALGAE;
  }

  public Supplier<Command> getScoreCommand() {
    return () -> {
      switch (Superstructure.getCurrentState()) {
        case BARGE_NET:
        case HOME:
        case FEEDING:
          return wristRollers
              .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
              .finallyDo(() -> WristRollers.resetBeam());
        case REEF_L2:
        case REEF_L3:
        case REEF_L4:
          return wristRollers
              .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
              .finallyDo(() -> WristRollers.resetBeam());
        case TROUGH:
          return wristRollers.applyGoalCommand(WristRollersGoal.TROUGH);
        case PROCESSOR:
          return wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE);
        case INTAKE:
        case REEF_ALGAE_L1:
        case REEF_ALGAE_L2:
        case TROUGH_FEED:
          return Commands.none();
      }
      return Commands.none();
    };
  }

  public ScoringLocation getTargetScoringLocation() {
    if (Controlboard.driveController.getHID().getLeftStickButton()) {
      return ScoringLocation.CENTER;
    } else if (Controlboard.driveController.getHID().getLeftBumperButton()) {
      return ScoringLocation.LEFT;
    } else {
      return ScoringLocation.RIGHT;
    }
  }

  public Pair<Pose2d, Pose2d> getTargetBranchPoses(ScoringLocation location) {
    if (getReefZone().isEmpty()) {
      return Pair.of(drivetrain.getEstimatedPose(), drivetrain.getEstimatedPose());
    }
    return location == ScoringLocation.RIGHT
        ? Pair.of(
            AllianceFlipUtil.get(
                    FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                .get(getReefZone().getAsInt())
                .getFirst(),
            AllianceFlipUtil.get(
                    FieldConstants.BLUE_PRE_REEF_LOCATIONS, FieldConstants.RED_PRE_REEF_LOCATIONS)
                .get(getReefZone().getAsInt())
                .getFirst())
        : Pair.of(
            AllianceFlipUtil.get(
                    FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                .get(getReefZone().getAsInt())
                .getSecond(),
            AllianceFlipUtil.get(
                    FieldConstants.BLUE_PRE_REEF_LOCATIONS, FieldConstants.RED_PRE_REEF_LOCATIONS)
                .get(getReefZone().getAsInt())
                .getSecond());
  }

  public Pair<Pose2d, SuperstructureState> getTargetAlgaeState() {
    var pair =
        AllianceFlipUtil.get(
                FieldConstants.BLUE_ALGAE_LOCATIONS, FieldConstants.RED_ALGAE_LOCATIONS)
            .get(getReefZone().getAsInt());
    return Pair.of(pair.getFirst(), mapAlgaeLevelToState(pair.getSecond()));
  }

  public SuperstructureState mapAlgaeLevelToState(AlgaeLevel algaeLevel) {
    return algaeLevel == AlgaeLevel.L1
        ? SuperstructureState.REEF_ALGAE_L1
        : SuperstructureState.REEF_ALGAE_L2;
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getEstimatedPose().getTranslation());
  }

  public Rotation2d getStationAlignAngle() {
    Rotation2d angle;
    if (drivetrain.getEstimatedPose().getY() <= FieldConstants.FIELD_DIMENSIONS.getY() / 2.0) {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(54), Rotation2d.fromDegrees(126));
    } else {
      angle = AllianceFlipUtil.get(Rotation2d.fromDegrees(-54), Rotation2d.fromDegrees(-126));
    }
    Logger.log("RobotState/StationAlignAngle", angle.getDegrees());
    return angle;
  }

  public void logTelemetry() {
    /* getReefZone()
    .ifPresent(
        reefZone -> {
          Logger.log(
              "Field/ScoringLocations",
              new Pose2d[] {
                FieldConstants.RED_REEF_LOCATIONS_FLIPPED.get(reefZone).getFirst(),
                FieldConstants.RED_REEF_LOCATIONS_FLIPPED.get(reefZone).getSecond(),
              },
              1.0);
          Logger.log("Field/ReefZone", reefZone, 1.0);
        }); */
    // Logger.log("Controls/ScoringSide", getTargetScoringLocation());
    if (Robot.isSimulation()) {
      visualizeComponents();
    }
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
                  - (activeCoral.get() == Coral.CORAL ? 0 : 4),
              wrist.getAngle()),
          ComponentVisualizer.getAlgaePose(
              elevator.getMeasuredHeight().getMeters()
                  - (activeCoral.get() == Coral.ALGAE ? 0 : 4),
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
