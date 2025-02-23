package frc2025;

import com.team4522.DashboardBoolean;
import com.team4522.DashboardNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2025.RobotState.GamePiece;
import frc2025.logging.Logger;
import java.util.function.Supplier;

public class Dashboard {

  private static final String overrides = "Overrides";

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetVoltage;
  public static DashboardNumber elevatorVoltage;
  public static DashboardNumber wristVoltage;
  public static DashboardNumber climberVoltage;
  public static DashboardNumber wristRollersVoltage;

  public static DashboardBoolean disableVisionUpdates;

  public static DashboardBoolean disableLockToReef;

  public static DashboardBoolean zeroElevator;

  private static Field2d field = new Field2d();

  static {
    initialize();
  }

  private static void initialize() {
    manualMode = new DashboardBoolean(overrides, "Manual Mode", false);
    resetVoltage = new DashboardBoolean(overrides, "Reset Voltage", false);
    elevatorVoltage = new DashboardNumber(overrides, "Elevator Voltage", 0);
    wristVoltage = new DashboardNumber(overrides, "Wrist Voltage", 0);
    climberVoltage = new DashboardNumber(overrides, "Climber Voltage", 0);
    wristRollersVoltage = new DashboardNumber(overrides, "Rollers Voltage", 0);

    disableVisionUpdates = new DashboardBoolean(overrides, "Disable Vision Updates", false);

    disableLockToReef = new DashboardBoolean(overrides, "Disable Lock To Reef", false);

    zeroElevator = new DashboardBoolean(overrides, "Zero Elevator", false);
  }

  public static void resetVoltageOverrides() {
    elevatorVoltage.set(0);
    wristVoltage.set(0);
    climberVoltage.set(0);
    wristRollersVoltage.set(0);
  }

  public static void publishPose(Pose2d estimate) {
    field.setRobotPose(estimate);
  }

  public static void periodic() {
    // SmartDashboard.putData("Field", field);

    if (Robot.isSimulation()) {
      Sim.periodic();
    }
  }

  protected class Sim {
    private static final SendableChooser<GamePiece> overridePieceChooser = new SendableChooser<>();

    static {
      overridePieceChooser.setDefaultOption("CORAL", GamePiece.CORAL);
      overridePieceChooser.addOption("NONE", GamePiece.NONE);
      overridePieceChooser.addOption("ALGAE", GamePiece.ALGAE);
      initialize();
    }

    private static GamePiece currentGamePiece = GamePiece.CORAL;
    private static Supplier<GamePiece> wantedGamePiece = () -> overridePieceChooser.getSelected();
    private static GamePiece lastGamePiece = wantedGamePiece.get();

    private static void initialize() {
      SmartDashboard.putData("Override Game Piece", overridePieceChooser);
    }

    public static void setGamePiece(GamePiece gamePiece) {
      currentGamePiece = gamePiece;
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
}
