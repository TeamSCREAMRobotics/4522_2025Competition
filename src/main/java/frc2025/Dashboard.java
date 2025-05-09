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
  private static final String vision = "Vision";
  private static final String tuning = "Tuning";

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetVoltage;
  public static DashboardNumber elevatorVoltage;
  public static DashboardNumber wristVoltage;
  public static DashboardNumber climberVoltage;
  public static DashboardNumber wristRollersVoltage;
  public static DashboardNumber funnelServoPosition;
  public static DashboardNumber climbRollersVoltage;
  public static DashboardBoolean disableAllVisionUpdates;
  public static DashboardBoolean disableAutoFeatures;
  public static DashboardBoolean zeroElevator;
  public static DashboardBoolean fieldCentric;
  public static DashboardBoolean disableCoralRequirement;
  // public static DashboardBoolean disableMegatag2;
  public static DashboardBoolean zeroClimber;
  public static SendableChooser<AlgaeLevel> wantedAlgaeLevel = new SendableChooser<>();
  public static DashboardBoolean disableClimber;
  public static DashboardBoolean unjam;
  public static DashboardNumber rotationOverride;
  public static DashboardBoolean submitRotationOverride;
  public static DashboardBoolean useGlobalEstimateForAutoAlign;
  public static DashboardBoolean coastClimber;
  public static DashboardBoolean coastElevator;
  public static DashboardBoolean disableAmbiguityRejection;

  public static DashboardNumber autoScoreDistance;

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
    climbRollersVoltage = new DashboardNumber(overrides, "Climb Rollers Volatage", 0.0);
    disableAllVisionUpdates = new DashboardBoolean(vision, "Disable Vision Updates", false);
    // disableMegatag2 = new DashboardBoolean(overrides, "Disable MegaTag2", false);
    disableAutoFeatures = new DashboardBoolean(overrides, "Disable Auto Features", false);
    disableCoralRequirement = new DashboardBoolean(overrides, "Disable Coral Requirement", false);
    zeroElevator = new DashboardBoolean(overrides, "Zero Elevator", false);
    zeroClimber = new DashboardBoolean(overrides, "Zero Climber", false);
    fieldCentric = new DashboardBoolean(overrides, "Field Centric", true);
    disableClimber = new DashboardBoolean(overrides, "Disable Climber", false);
    unjam = new DashboardBoolean(overrides, "Unjam", false);
    rotationOverride = new DashboardNumber(overrides, "Reset Rotation", 0.0);
    submitRotationOverride = new DashboardBoolean(overrides, "Submit Rotation Override", false);
    useGlobalEstimateForAutoAlign = new DashboardBoolean(vision, "Global Estimate For AA", false);
    coastClimber = new DashboardBoolean(overrides, "Coast Climber", false);
    coastElevator = new DashboardBoolean(overrides, "Coast Elevator", false);
    disableAmbiguityRejection = new DashboardBoolean(vision, "Disable Ambiguity Rejection", false);

    autoScoreDistance = new DashboardNumber(tuning, "Auto Score Distance", 0.03);

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
    climbRollersVoltage.set(0);
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
