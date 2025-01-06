package frc2025.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.drivetrain.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;
import lombok.Getter;
import util.AllianceFlipUtil;
import util.RunnableUtil.RunOnce;
import util.ScreamUtil;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private double lastSimTime;

  private RunOnce operatorPerspectiveApplier = new RunOnce();

  @Getter private final PhoenixSwerveHelper helper;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    helper =
        new PhoenixSwerveHelper(
            this::getPose,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        () -> getState().Speeds,
        (speeds, feedforwards) ->
            setControl(
                helper
                    .getApplyRobotSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        DrivetrainConstants.PATH_FOLLOWING_CONTROLLER,
        DrivetrainConstants.ROBOT_CONFIG,
        () -> false,
        this);

    System.out.println("[Init] Drivetrain initialization complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    return run(() -> {
          var xSpeed =
              DrivetrainConstants.X_ALIGNMENT_CONTROLLER.calculate(
                  getPose().getTranslation().getX(), pose.get().getX());
          var ySpeed =
              DrivetrainConstants.Y_ALIGNMENT_CONTROLLER.calculate(
                  getPose().getTranslation().getY(), pose.get().getY());
          var headingSpeed =
              DrivetrainConstants.HEADING_CONTROLLER.calculate(
                  getPose().getRotation().getRadians(), pose.get().getRotation().getRadians());
          setControl(helper.getApplyFieldSpeeds(new ChassisSpeeds(xSpeed, ySpeed, headingSpeed)));
        })
        .beforeStarting(
            () -> {
              DrivetrainConstants.X_ALIGNMENT_CONTROLLER.reset(getPose().getX());
              DrivetrainConstants.Y_ALIGNMENT_CONTROLLER.reset(getPose().getY());
              DrivetrainConstants.HEADING_CONTROLLER.reset();
            });
  }

  public Command driveToBargeScoringZone(Supplier<Translation2d> translation) {
    return applyRequest(
            () ->
                helper.getFacingAngle(
                    new Translation2d(
                        DrivetrainConstants.X_ALIGNMENT_CONTROLLER.calculate(
                                getPose().getX(),
                                AllianceFlipUtil.get(
                                    FieldConstants.BLUE_BARGE_ALIGN_X,
                                    FieldConstants.RED_BARGE_ALIGN_X))
                            * AllianceFlipUtil.getDirectionCoefficient(),
                        translation.get().getY()),
                    AllianceFlipUtil.getFwdHeading()))
        .beforeStarting(
            () ->
                DrivetrainConstants.X_ALIGNMENT_CONTROLLER.reset(
                    getPose().getX(),
                    getLinearVelocity().getNorm() * AllianceFlipUtil.getDirectionCoefficient()));
  }

  public void updateSimState() {
    final double currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    updateSimState(deltaTime, RobotController.getBatteryVoltage());
  }

  public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold) {
    return ScreamUtil.withinAngleThreshold(targetAngle, getHeading(), threshold);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    for (int i = 0; i < DrivetrainConstants.NUM_MODULES; i++) {
      states[i] = getModules()[i].getCurrentState();
    }
    return states;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  public Translation2d getLinearVelocity() {
    return new Translation2d(
            getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond)
        .rotateBy(getHeading());
  }

  public Twist2d getFieldVelocity() {
    return new Twist2d(
        getLinearVelocity().getX(),
        getLinearVelocity().getY(),
        getState().Speeds.omegaRadiansPerSecond);
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  @Override
  public void periodic() {
    attemptToSetPerspective();
  }

  public void attemptToSetPerspective() {
    operatorPerspectiveApplier.runOnceWhenTrueThenWhenChanged(
        () -> setOperatorPerspectiveForward(AllianceFlipUtil.getFwdHeading()),
        DriverStation.getAlliance().isPresent(),
        DriverStation.getAlliance().orElse(null));
  }
}
