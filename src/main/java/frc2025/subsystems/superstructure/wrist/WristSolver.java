package frc2025.subsystems.superstructure.wrist;

import data.Length;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructurePosition;

public class WristSolver {
  public record Solution(Rotation2d wristAngle, Length height) {
    @Override
    public final String toString() {
      return "(" + wristAngle.getDegrees() + ", " + height.getInches() + ")";
    }
  }

  public static Solution solve(SuperstructurePosition position) {
    double armLength = WristConstants.MANIPULATOR_LENGTH.getMeters();
    double horizontalDistance = Math.abs(position.position.getX());
    double verticalOffset =
        Math.sqrt(armLength * armLength - horizontalDistance * horizontalDistance);

    // Calculate base angle using atan2
    double baseAngle = Math.atan2(position.position.getX(), verticalOffset);

    // Adjust height based on elbow configuration
    double adjustedHeight;
    if (!position.elbowDown) {
      adjustedHeight = position.position.getY() - verticalOffset;
      baseAngle = position.position.getX() >= 0 ? baseAngle : -baseAngle;
    } else {
      adjustedHeight = position.position.getY() + verticalOffset;
      baseAngle = position.position.getX() >= 0 ? Math.PI - baseAngle : -Math.PI + baseAngle;
    }
    return new Solution(Rotation2d.fromRadians(baseAngle), Length.fromMeters(adjustedHeight));
  }
}
