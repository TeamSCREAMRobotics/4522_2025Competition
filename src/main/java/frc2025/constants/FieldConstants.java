package frc2025.constants;

import data.Length;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.Map;
import zones.HexagonalPoseArea;
import zones.RectangularPoseArea;

public class FieldConstants {

  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(17.548225, 8.0518);

  public static final RectangularPoseArea FIELD_AREA =
      new RectangularPoseArea(Translation2d.kZero, FIELD_DIMENSIONS);

  public enum AlgaeLevel {
    L1,
    L2;
  }

  public static final Length CORAL_DIAMETER = Length.fromInches(4.5);

  public static final Length BRANCH_TO_REEF_EDGE = Length.fromInches(2.111249);

  public static final double STATION_X_OFFSET = 0.1;
  public static final double STATION_Y_OFFSET = 0.1;

  public static final Pose2d BLUE_PROCESSOR_FEEDER_ALIGN =
      new Pose2d(1.544 - STATION_X_OFFSET, 0.734 - STATION_Y_OFFSET, Rotation2d.fromDegrees(54.0));
  public static final Pose2d RED_PROCESSOR_FEEDER_ALIGN =
      new Pose2d(
          FIELD_DIMENSIONS.getX() - 1.544 + STATION_X_OFFSET,
          FIELD_DIMENSIONS.getY() - 0.734 + STATION_Y_OFFSET,
          Rotation2d.fromDegrees(54.0 + 180.0));

  public static final Pose2d BLUE_NONPROCESSOR_FEEDER_ALIGN =
      new Pose2d(1.544 - STATION_X_OFFSET, 7.293 + STATION_Y_OFFSET, Rotation2d.fromDegrees(54.0));
  public static final Pose2d RED_NONPROCESSOR_FEEDER_ALIGN =
      new Pose2d(
          FIELD_DIMENSIONS.getX() - 1.544 + STATION_X_OFFSET,
          FIELD_DIMENSIONS.getY() - 0.734 - STATION_Y_OFFSET,
          Rotation2d.fromDegrees(-54.0 + 180.0));

  public static final Pose2d BLUE_BARGE_ALIGN =
      new Pose2d(7.7, FIELD_DIMENSIONS.getY() * /* 0.75 */ 0.65, Rotation2d.kZero);
  public static final Pose2d RED_BARGE_ALIGN =
      new Pose2d(
          FIELD_DIMENSIONS.getX() - 7.7,
          FIELD_DIMENSIONS.getY() * /* 0.25 */ 0.35,
          Rotation2d.k180deg);

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), FIELD_DIMENSIONS.getY() / 2.0);
  public static final Translation2d RED_REEF_CENTER = FIELD_DIMENSIONS.minus(BLUE_REEF_CENTER);

  public static final HexagonalPoseArea BLUE_REEF =
      new HexagonalPoseArea(BLUE_REEF_CENTER, Length.fromMeters(10), Rotation2d.fromDegrees(-30));
  public static final HexagonalPoseArea RED_REEF =
      new HexagonalPoseArea(RED_REEF_CENTER, Length.fromMeters(10), Rotation2d.fromDegrees(-30));

  public static final Map<Integer, Pair<Pose2d, Pose2d>> BLUE_PRE_REEF_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, Pose2d>> RED_PRE_REEF_LOCATIONS = new HashMap<>();

  public static final Map<Integer, Pair<Pose2d, Pose2d>> BLUE_REEF_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, Pose2d>> RED_REEF_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, Pose2d>> RED_REEF_LOCATIONS_FLIPPED =
      new HashMap<>();

  public static final Map<Integer, Pair<Pose2d, AlgaeLevel>> BLUE_ALGAE_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, AlgaeLevel>> RED_ALGAE_LOCATIONS = new HashMap<>();

  public static final int[] ALL_TAGS =
      new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
  public static final int[] BLUE_REEF_TAGS = new int[] {22, 21, 20, 19, 18, 17};
  public static final int[] RED_REEF_TAGS = new int[] {11, 10, 9, 8, 7, 6};
  public static final int[] REEF_TAGS = new int[] {22, 21, 20, 19, 18, 17, 11, 10, 9, 8, 7, 6};

  private static Pair<Integer, Pose2d> getTagPair(int id) {
    return Pair.of(
        id,
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(id)
            .get()
            .toPose2d());
  }

  public static final Translation2d REEF_CENTER_TO_TOP_BRANCH =
      new Translation2d(Units.inchesToMeters(30.738196), Units.inchesToMeters(6.633604));
  public static final Translation2d REEF_CENTER_TO_BOTTOM_BRANCH =
      new Translation2d(Units.inchesToMeters(30.738196), -Units.inchesToMeters(6.633604));

  public static final Translation2d BRANCH_TO_ROBOT =
      new Translation2d(Units.inchesToMeters(17.3075 + 0.1 + BRANCH_TO_REEF_EDGE.getInches()), 0.0);

  public static final Translation2d PRE_REEF_OFFSET =
      new Translation2d(Units.inchesToMeters(10), 0.0);

  public static final Translation2d SCORE_LOCATION_1 =
      REEF_CENTER_TO_TOP_BRANCH.plus(BRANCH_TO_ROBOT);
  public static final Translation2d SCORE_LOCATION_2 =
      REEF_CENTER_TO_BOTTOM_BRANCH.plus(BRANCH_TO_ROBOT);
  public static final Translation2d ALGAE_LOCATION = new Translation2d(SCORE_LOCATION_1.getX(), 0);

  static {
    for (int i = 0; i < 6; i++) {
      Rotation2d rotation = Rotation2d.fromDegrees(i * 60);
      Rotation2d poseRotation = Rotation2d.fromDegrees((i * 60) + 180);

      BLUE_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(BLUE_REEF_CENTER.plus(SCORE_LOCATION_1.rotateBy(rotation)), poseRotation),
              new Pose2d(
                  BLUE_REEF_CENTER.plus(SCORE_LOCATION_2.rotateBy(rotation)), poseRotation)));

      BLUE_PRE_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(
                  BLUE_REEF_CENTER.plus(SCORE_LOCATION_1.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation),
              new Pose2d(
                  BLUE_REEF_CENTER.plus(SCORE_LOCATION_2.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation)));

      RED_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(RED_REEF_CENTER.plus(SCORE_LOCATION_1.rotateBy(rotation)), poseRotation),
              new Pose2d(RED_REEF_CENTER.plus(SCORE_LOCATION_2.rotateBy(rotation)), poseRotation)));

      RED_PRE_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(
                  RED_REEF_CENTER.plus(SCORE_LOCATION_1.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation),
              new Pose2d(
                  RED_REEF_CENTER.plus(SCORE_LOCATION_2.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation)));

      BLUE_ALGAE_LOCATIONS.put(
          i,
          Pair.of(
              new Pose2d(BLUE_REEF_CENTER.plus(ALGAE_LOCATION.rotateBy(rotation)), poseRotation),
              getAlgaeLevel(i)));

      RED_ALGAE_LOCATIONS.put(
          i,
          Pair.of(
              new Pose2d(RED_REEF_CENTER.plus(ALGAE_LOCATION.rotateBy(rotation)), poseRotation),
              getAlgaeLevel(i + 1)));
    }

    for (int i = 0; i < 6; i++) {
      Rotation2d rotation = Rotation2d.fromDegrees((i * 60) + 180);
      Rotation2d poseRotation = Rotation2d.fromDegrees((i * 60));

      RED_REEF_LOCATIONS_FLIPPED.put(
          i,
          new Pair<>(
              new Pose2d(
                  RED_REEF_CENTER.plus(SCORE_LOCATION_1.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation),
              new Pose2d(
                  RED_REEF_CENTER.plus(SCORE_LOCATION_2.plus(PRE_REEF_OFFSET).rotateBy(rotation)),
                  poseRotation)));
    }
  }

  private static AlgaeLevel getAlgaeLevel(int zone) {
    zone %= 2;
    return zone == 0 ? AlgaeLevel.L1 : AlgaeLevel.L2;
  }
}
