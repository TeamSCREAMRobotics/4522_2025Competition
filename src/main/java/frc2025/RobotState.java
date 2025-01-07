package frc2025;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc2025.RobotContainer.Subsystems;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.subsystems.drivetrain.Drivetrain;
import java.util.OptionalInt;
import util.AllianceFlipUtil;

public class RobotState {
  private final Drivetrain drivetrain;

  /* private final Ligament elevator;
  private final Ligament pivot;

  private final Mechanism elevatorPivot;

  private final MechanismVisualizer visualizer; */

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getPose().getTranslation());
  }

  public void logTelemetry() {
    getReefZone()
        .ifPresent(
            reefZone ->
                DogLog.log(
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
    DogLog.log("Controls/ScoringSide", Controlboard.getScoringSide().get());
  }
}
