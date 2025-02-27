package frc2025;

import com.team4522.DashboardBoolean;
import com.team4522.DashboardNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Dashboard {

  private static final String overrides = "Overrides";

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetVoltage;
  public static DashboardNumber elevatorVoltage;
  public static DashboardNumber wristVoltage;
  public static DashboardNumber climberVoltage;
  public static DashboardNumber wristRollersVoltage;
  public static DashboardBoolean disableVisionUpdates;
  public static DashboardBoolean disableAutoFeatures;
  public static DashboardBoolean zeroElevator;
  public static DashboardBoolean fieldCentric;

  private static Field2d field = new Field2d();

  static {
    initialize();
    if (Robot.isSimulation()) {
      Sim.initialize();
    }
  }

  private static void initialize() {
    manualMode = new DashboardBoolean(overrides, "Manual Mode", false);
    resetVoltage = new DashboardBoolean(overrides, "Reset Voltage", false);
    elevatorVoltage = new DashboardNumber(overrides, "Elevator Voltage", 0);
    wristVoltage = new DashboardNumber(overrides, "Wrist Voltage", 0);
    climberVoltage = new DashboardNumber(overrides, "Climber Voltage", 0);
    wristRollersVoltage = new DashboardNumber(overrides, "Rollers Voltage", 0);
    disableVisionUpdates = new DashboardBoolean(overrides, "Disable Vision Updates", false);
    disableAutoFeatures = new DashboardBoolean(overrides, "Disable Auto Features", false);
    zeroElevator = new DashboardBoolean(overrides, "Zero Elevator", false);
    fieldCentric = new DashboardBoolean(overrides, "Field Centric", true);
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
  }

  public static class Sim {

    public static DashboardBoolean hasCoral;

    private static void initialize() {
      hasCoral = new DashboardBoolean(overrides, "Has Coral", true);
    }

    public static void periodic() {}
  }
}
