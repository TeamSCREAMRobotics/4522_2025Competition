package frc2025;

import com.team4522.DashboardBoolean;
import com.team4522.DashboardNumber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2025.constants.FieldConstants.AlgaeLevel;

public class Dashboard {

  private static final String overrides = "Overrides";

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetVoltage;
  public static DashboardNumber elevatorVoltage;
  public static DashboardNumber wristVoltage;
  public static DashboardNumber climberVoltage;
  public static DashboardNumber wristRollersVoltage;
  public static DashboardNumber funnelServoPosition;
  public static DashboardNumber latchServoPosition;
  public static DashboardBoolean disableAllVisionUpdates;
  public static DashboardBoolean disableAutoFeatures;
  public static DashboardBoolean zeroElevator;
  public static DashboardBoolean fieldCentric;
  public static DashboardBoolean disableCoralRequirement;
  public static DashboardBoolean disableMegatag2;
  public static DashboardBoolean zeroClimber;
  public static SendableChooser<AlgaeLevel> wantedAlgaeLevel = new SendableChooser<>();

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
    funnelServoPosition = new DashboardNumber(overrides, "Funnel Servo Position", 1);
    latchServoPosition = new DashboardNumber(overrides, "Latch Servo Position", 1);
    disableAllVisionUpdates = new DashboardBoolean(overrides, "Disable Vision Updates", false);
    disableMegatag2 = new DashboardBoolean(overrides, "Disable MegaTag2", false);
    disableAutoFeatures = new DashboardBoolean(overrides, "Disable Auto Features", false);
    disableCoralRequirement = new DashboardBoolean(overrides, "Disable Coral Requirement", false);
    zeroElevator = new DashboardBoolean(overrides, "Zero Elevator", false);
    zeroClimber = new DashboardBoolean(overrides, "Zero Climber", false);
    fieldCentric = new DashboardBoolean(overrides, "Field Centric", true);

    for (AlgaeLevel level : AlgaeLevel.values()) {
      wantedAlgaeLevel.addOption(level.toString(), level);
    }
    wantedAlgaeLevel.setDefaultOption(AlgaeLevel.L1.name(), AlgaeLevel.L1);

    SmartDashboard.putData("Wanted Algae Level", wantedAlgaeLevel);
  }

  public static void resetManualOverrides() {
    elevatorVoltage.set(0);
    wristVoltage.set(0);
    climberVoltage.set(0);
    wristRollersVoltage.set(0);
    funnelServoPosition.set(1);
    latchServoPosition.set(1);
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
