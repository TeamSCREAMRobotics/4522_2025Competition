package frc2025;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2025.RobotState.GamePiece;
import frc2025.logging.Logger;
import java.util.function.Supplier;

public class Dashboard {
  private static final SendableChooser<GamePiece> overridePieceChooser = new SendableChooser<>();

  static {
    overridePieceChooser.setDefaultOption("NONE", GamePiece.NONE);
    overridePieceChooser.addOption("CORAL", GamePiece.CORAL);
    overridePieceChooser.addOption("ALGAE", GamePiece.ALGAE);
    initialize();
  }

  public static GamePiece currentGamePiece = GamePiece.NONE;
  private static Supplier<GamePiece> wantedGamePiece = () -> overridePieceChooser.getSelected();
  private static GamePiece lastGamePiece = wantedGamePiece.get();

  private static void initialize() {
    SmartDashboard.putData("Override Game Piece", overridePieceChooser);
  }

  public static GamePiece selectedGamePiece() {
    return currentGamePiece;
  }

  public static void periodic() {
    Logger.log("RobotState/Active Game Piece", currentGamePiece.toString());

    if (wantedGamePiece.get() != lastGamePiece) {
      currentGamePiece = wantedGamePiece.get();
    }

    lastGamePiece = wantedGamePiece.get();
  }
}
