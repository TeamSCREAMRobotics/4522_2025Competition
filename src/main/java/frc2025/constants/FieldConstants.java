package frc2025.constants;

import data.Length;
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

  public static final Length BRANCH_TO_REEF_EDGE = Length.fromInches(2.111249);

  public static final Pose2d BLUE_BARGE_ALIGN =
      new Pose2d(7.715, FIELD_DIMENSIONS.getY() * 0.75, Rotation2d.fromDegrees(0));
  public static final Pose2d RED_BARGE_ALIGN =
      new Pose2d(
          FIELD_DIMENSIONS.getX() - 7.715,
          FIELD_DIMENSIONS.getY() * 0.25,
          Rotation2d.fromDegrees(180));

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), FIELD_DIMENSIONS.getY() / 2.0);
  public static final Translation2d RED_REEF_CENTER = FIELD_DIMENSIONS.minus(BLUE_REEF_CENTER);

  public static final HexagonalPoseArea BLUE_REEF =
      new HexagonalPoseArea(BLUE_REEF_CENTER, Length.fromMeters(5), Rotation2d.fromDegrees(-30));
  public static final HexagonalPoseArea RED_REEF =
      new HexagonalPoseArea(RED_REEF_CENTER, Length.fromMeters(5), Rotation2d.fromDegrees(150));

  public static final Map<Integer, Pair<Pose2d, Pose2d>> BLUE_REEF_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pair<Pose2d, Pose2d>> RED_REEF_LOCATIONS = new HashMap<>();

  public static final Map<Integer, Pose2d> BLUE_ALGAE_LOCATIONS = new HashMap<>();
  public static final Map<Integer, Pose2d> RED_ALGAE_LOCATIONS = new HashMap<>();

  public static final Translation2d REEF_CENTER_TO_TOP_BRANCH =
      new Translation2d(Units.inchesToMeters(30.738196), Units.inchesToMeters(6.633604));
  public static final Translation2d REEF_CENTER_TO_BOTTOM_BRANCH =
      new Translation2d(Units.inchesToMeters(30.738196), -Units.inchesToMeters(6.633604));

  public static final Translation2d BRANCH_TO_ROBOT =
      new Translation2d(Units.inchesToMeters(17.307500 + BRANCH_TO_REEF_EDGE.getInches()), 0);

  public static final Translation2d SCORE_LOCATION_1 =
      REEF_CENTER_TO_TOP_BRANCH.plus(BRANCH_TO_ROBOT);
  public static final Translation2d SCORE_LOCATION_2 =
      REEF_CENTER_TO_BOTTOM_BRANCH.plus(BRANCH_TO_ROBOT);
  public static final Translation2d ALGAE_LOCATION =
      new Translation2d(
          5.268944 + BRANCH_TO_ROBOT.getX(),
          (SCORE_LOCATION_1.getY() + SCORE_LOCATION_2.getY()) / 2.0);

  static {
    Translation2d blueAlgaeLocation = ALGAE_LOCATION.minus(BLUE_REEF_CENTER);
    Translation2d redAlgaeLocation = FIELD_DIMENSIONS.minus(ALGAE_LOCATION).minus(RED_REEF_CENTER);

    for (int i = 0; i < 6; i++) {
      Rotation2d rotation = Rotation2d.fromDegrees(i * 60);
      Rotation2d blueRotation = Rotation2d.fromDegrees(-180 + (i * 60));
      Rotation2d redRotation = Rotation2d.fromDegrees(i * 60);

      BLUE_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(BLUE_REEF_CENTER.plus(SCORE_LOCATION_1.rotateBy(rotation)), blueRotation),
              new Pose2d(
                  BLUE_REEF_CENTER.plus(SCORE_LOCATION_2.rotateBy(rotation)), blueRotation)));

      RED_REEF_LOCATIONS.put(
          i,
          new Pair<>(
              new Pose2d(RED_REEF_CENTER.minus(SCORE_LOCATION_1.rotateBy(rotation)), redRotation),
              new Pose2d(RED_REEF_CENTER.minus(SCORE_LOCATION_2.rotateBy(rotation)), redRotation)));

      BLUE_ALGAE_LOCATIONS.put(
          i, new Pose2d(blueAlgaeLocation.rotateBy(rotation).plus(BLUE_REEF_CENTER), blueRotation));

      RED_ALGAE_LOCATIONS.put(
          i, new Pose2d(redAlgaeLocation.rotateBy(rotation).plus(RED_REEF_CENTER), redRotation));
    }
  }
}
