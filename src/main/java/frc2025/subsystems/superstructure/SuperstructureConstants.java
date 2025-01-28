package frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureConstants {
  public enum Orientation {
    ELBOW_DOWN,
    ELBOW_UP;
  }

  public static final Translation2d SCORE_OFFSET =
      new Translation2d(Units.inchesToMeters(1.5), Units.inchesToMeters(1.5));

  public enum SuperstructurePosition {
    HOME(new Translation2d(0, WristConstants.MANIPULATOR_LENGTH.getMeters()), Orientation.ELBOW_UP),
    IDLE(
        new Translation2d(Units.inchesToMeters(10.71), Units.inchesToMeters(27.31)),
        Orientation.ELBOW_UP),
    REEF_L2(FieldConstants.BRANCH_L2.plus(SCORE_OFFSET), Orientation.ELBOW_DOWN),
    REEF_L3(FieldConstants.BRANCH_L3.plus(SCORE_OFFSET), Orientation.ELBOW_DOWN),
    REEF_L4(FieldConstants.BRANCH_L4.plus(SCORE_OFFSET), Orientation.ELBOW_DOWN),
    PRE_IDLE(
        IDLE.position.plus(new Translation2d(0, Units.inchesToMeters(10.0))), Orientation.ELBOW_UP);

    public final Translation2d position;
    public final Orientation orientation;

    private SuperstructurePosition(Translation2d position, Orientation orientation) {
      this.position = position;
      this.orientation = orientation;
    }
  }
}
