package frc2025.subsystems.superstructure.wrist;

import data.Length;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2025.RobotState;
import frc2025.subsystems.superstructure.SuperstructureConstants.Orientation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructurePosition;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;

public class WristSolver {
  public record Solution(Rotation2d wristAngle, Length height) {
    @Override
    public final String toString() {
      return "(" + wristAngle.getDegrees() + ", " + height.getInches() + ")";
    }
  }

  public static Translation2d forward(Rotation2d wristAngle, Length elevatorHeight) {
    Translation2d t1 = new Translation2d(WristConstants.MANIPULATOR_LENGTH.getMeters(), wristAngle);
    Translation2d t2 =
        new Translation2d(
            0.0, elevatorHeight.plus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR).getMeters());
    return t2.plus(t1);
  }

  public static Solution inverse(
      Translation2d target,
      Translation2d finalTarget,
      Orientation preferredOrientation,
      Rotation2d currentWristTheta) {
    double a = WristConstants.MANIPULATOR_LENGTH.getMeters();

    boolean atFinalTarget = target.getDistance(finalTarget) < 0.05;

    if (atFinalTarget && preferredOrientation != null) {
      return calculateSolution(target, a, preferredOrientation);
    }

    Solution elbowDownSolution = calculateSolution(target, a, Orientation.ELBOW_DOWN);
    Solution elbowUpSolution = calculateSolution(target, a, Orientation.ELBOW_UP);

    double elbowDownDiff =
        Math.abs(elbowDownSolution.wristAngle.minus(currentWristTheta).getRadians());
    double elbowUpDiff = Math.abs(elbowUpSolution.wristAngle.minus(currentWristTheta).getRadians());

    return elbowDownDiff < elbowUpDiff ? elbowDownSolution : elbowUpSolution;
  }

  public static Solution inverse(Translation2d target, Orientation orientation) {
    return calculateSolution(target, WristConstants.MANIPULATOR_LENGTH.getMeters(), orientation);
  }

  public static Solution inverse(SuperstructurePosition position) {
    return inverse(position.position, position.orientation);
  }

  private static SlewRateLimiter heightRateLimiter = new SlewRateLimiter(2.5);

  private static Solution calculateSolution(
      Translation2d target, double armLength, Orientation orientation) {
    double thetaRads =
        orientation == Orientation.ELBOW_DOWN
            ? -Math.acos(target.getX() / armLength)
            : Math.acos(target.getX() / armLength);

    double z1 = Math.sqrt(Math.pow(armLength, 2) - Math.pow(target.getX(), 2));
    double height = orientation == Orientation.ELBOW_DOWN ? target.getY() + z1 : target.getY() - z1;

    RobotState.setAxesPose(new Pose3d(target.getX(), 0, target.getY(), Rotation3d.kZero));

    return new Solution(
        Rotation2d.fromRadians(thetaRads),
        Length.fromMeters(heightRateLimiter.calculate(height))
            .minus(ElevatorConstants.MIN_HEIGHT_FROM_FLOOR));
  }
}
