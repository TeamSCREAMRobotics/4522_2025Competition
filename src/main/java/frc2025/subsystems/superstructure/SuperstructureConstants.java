package frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureConstants {
  public static final Translation2d SCORE_OFFSET =
      new Translation2d(Units.inchesToMeters(1.5), Units.inchesToMeters(1.5));

  public enum SuperstructurePosition {
    HOME(new Translation2d(0, WristConstants.MANIPULATOR_LENGTH.getMeters()), false),
    IDLE(new Translation2d(Units.inchesToMeters(10.71), Units.inchesToMeters(27.31)), false),
    REEF_L2(FieldConstants.BRANCH_L2.plus(SCORE_OFFSET), true),
    REEF_L3(FieldConstants.BRANCH_L3.plus(SCORE_OFFSET), true),
    REEF_L4(FieldConstants.BRANCH_L4.plus(SCORE_OFFSET), true);

    public final Translation2d position;
    public final boolean elbowDown;

    private SuperstructurePosition(Translation2d position, boolean elbowDown) {
      this.position = position;
      this.elbowDown = elbowDown;
    }
  }
}
