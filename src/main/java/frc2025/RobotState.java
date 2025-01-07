package frc2025;

import dashboard.Mechanism;
import dashboard.MechanismVisualizer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc2025.RobotContainer.Subsystems;
import frc2025.constants.FieldConstants;
import frc2025.constants.SimConstants;
import frc2025.controlboard.Controlboard;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.intake.IntakeDeploy;
import frc2025.subsystems.intake.IntakeRollers;
import frc2025.subsystems.wrist.Wrist;
import frc2025.subsystems.wrist.WristRollers;
import java.util.OptionalInt;
import util.AllianceFlipUtil;

public class RobotState {
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final Wrist wrist;
  private final WristRollers wristRollers;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRollers intakeRollers;

  private final Mechanism elevatorPivot;

  private final MechanismVisualizer visualizer;

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
    this.elevator = subsystems.elevator();
    this.wrist = subsystems.wrist();
    this.wristRollers = subsystems.wristRollers();
    this.intakeDeploy = subsystems.intakeDeploy();
    this.intakeRollers = subsystems.intakeRollers();

    elevatorPivot =
        new Mechanism("ElevatorWrist", elevator.getLigament(), wrist.getLigament())
            .withStaticPosition(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0));

    visualizer =
        new MechanismVisualizer(
            SimConstants.MEASURED_MECHANISM,
            SimConstants.SETPOINT_MECHANISM,
            RobotState::telemeterizeMechanisms,
            elevatorPivot);

    visualizer.setEnabled(true);
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getPose().getTranslation());
  }

  public void logTelemetry() {
    getReefZone()
        .ifPresent(
            reefZone ->
                Logger.log(
                    "Field/ScoringLocations",
                    new Pose2d[] {
                      AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(reefZone)
                          .getFirst(),
                      AllianceFlipUtil.get(
                              FieldConstants.BLUE_REEF_LOCATIONS, FieldConstants.RED_REEF_LOCATIONS)
                          .get(reefZone)
                          .getSecond()
                    }));
    Logger.log("Controls/ScoringSide", Controlboard.getScoringSide().get());
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }
}
