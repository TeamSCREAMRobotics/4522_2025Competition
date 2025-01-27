package frc2025.subsystems.superstructure.wrist;

import data.Length;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructurePosition;
import frc2025.subsystems.superstructure.elevator.ElevatorConstants;

public class WristSolver {
  public record Solution(Rotation2d wristAngle, Length height) {
    @Override
    public final String toString() {
      return "(" + wristAngle.getDegrees() + ", " + height.getInches() + ")";
    }
  }

  public static Solution solve(SuperstructurePosition position) {
    return new Solution(Rotation2d.kZero, Length.fromInches(0));
  }
}
