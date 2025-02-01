package frc2025.constants;

import data.Length;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc2025.subsystems.superstructure.wrist.WristConstants;
import java.util.HashMap;
import java.util.Map;
import zones.HexagonalPoseArea;

public class FieldConstants {

  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(17.548225, 8.0518);

  public static final Length BRANCH_DISTANCE = Length.fromInches(-17.807500);
  public static final Translation2d BRANCH_L2 =
      new Translation2d(BRANCH_DISTANCE.getMeters(), Units.inchesToMeters(31.875));
  public static final Translation2d BRANCH_L3 =
      new Translation2d(BRANCH_DISTANCE.getMeters(), Units.inchesToMeters(47.625));
  public static final Translation2d BRANCH_L4 =
      new Translation2d(BRANCH_DISTANCE.getMeters(), Units.inchesToMeters(72.0));

  public static final Pose2d BLUE_BARGE_ALIGN =
      new Pose2d(7.715, FIELD_DIMENSIONS.getY() * 0.75, Rotation2d.fromDegrees(0));
  public static final Pose2d RED_BARGE_ALIGN =
      new Pose2d(
          FIELD_DIMENSIONS.getX() - 7.715,
          FIELD_DIMENSIONS.getY() * 0.25,
          Rotation2d.fromDegrees(180));

  public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.489338, 4.034306);
  public static final Translation2d RED_REEF_CENTER = FIELD_DIMENSIONS.minus(BLUE_REEF_CENTER);

  public static final HexagonalPoseArea BLUE_REEF =
      new HexagonalPoseArea(BLUE_REEF_CENTER, Length.fromMeters(5), Rotation2d.fromDegrees(-30));
  public static final HexagonalPoseArea RED_REEF =
      new HexagonalPoseArea(RED_REEF_CENTER, Length.fromMeters(5), Rotation2d.fromDegrees(150));

  public static final Map<Integer, Pair<Pose2d, Pose2d>> BLUE_REEF_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, Pose2d>> RED_REEF_LOCATIONS = new HashMap<>();

  public static final Translation2d SCORE_LOCATION_1 =
      new Translation2d(5.268944 + 0.5059, 3.869997 - WristConstants.ROLLERS_TO_ORIGIN.getMeters());
  public static final Translation2d SCORE_LOCATION_2 =
      new Translation2d(5.268944 + 0.5059, 4.198614 - WristConstants.ROLLERS_TO_ORIGIN.getMeters());

  static {
    Translation2d blueScoreLocation1 = SCORE_LOCATION_1.minus(BLUE_REEF_CENTER);
    Translation2d blueScoreLocation2 = SCORE_LOCATION_2.minus(BLUE_REEF_CENTER);
    Translation2d redScoreLocation1 =
        FIELD_DIMENSIONS.minus(SCORE_LOCATION_1).minus(RED_REEF_CENTER);
    Translation2d redScoreLocation2 =
        FIELD_DIMENSIONS.minus(SCORE_LOCATION_2).minus(RED_REEF_CENTER);

    for (int i = 0; i < 6; i++) {
      Rotation2d rotation = Rotation2d.fromDegrees(i * 60);
      Rotation2d blueRotation = Rotation2d.fromDegrees(-180 + (i * 60)).plus(Rotation2d.k180deg);
      Rotation2d redRotation = Rotation2d.fromDegrees(i * 60).plus(Rotation2d.k180deg);

      BLUE_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(
                  blueScoreLocation1.rotateBy(rotation).plus(BLUE_REEF_CENTER), blueRotation),
              new Pose2d(
                  blueScoreLocation2.rotateBy(rotation).plus(BLUE_REEF_CENTER), blueRotation)));

      RED_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(redScoreLocation1.rotateBy(rotation).plus(RED_REEF_CENTER), redRotation),
              new Pose2d(redScoreLocation2.rotateBy(rotation).plus(RED_REEF_CENTER), redRotation)));
    }
  }
}
