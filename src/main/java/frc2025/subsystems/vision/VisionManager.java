package frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc2025.Dashboard;
import frc2025.Robot;
import frc2025.constants.Constants;
import frc2025.constants.FieldConstants;
import frc2025.logging.Logger;
import frc2025.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalInt;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import util.AllianceFlipUtil;
import util.GeomUtil;
import vision.LimelightHelpers;
import vision.LimelightHelpers.PoseEstimate;
import vision.LimelightVision.Limelight;

public class VisionManager {

  public static class Limelights {
    public static final Limelight REEF_LEFT =
        new Limelight(
            "limelight-reefb",
            new Pose3d(
                0.209727,
                0.270552,
                0.202070,
                new Rotation3d(0, -Units.degreesToRadians(20.0), -Units.degreesToRadians(30))));
    public static final Limelight REEF_RIGHT =
        new Limelight(
            "limelight-reefa",
            new Pose3d(
                0.213194,
                -0.268550,
                0.202070,
                new Rotation3d(0, -Units.degreesToRadians(20.0), Units.degreesToRadians(30))));
    public static final Limelight FRONT_ELEVATOR =
        new Limelight(
            "limelight-front",
            new Pose3d(
                Units.inchesToMeters(6.223684),
                Units.inchesToMeters(11.834409),
                Units.inchesToMeters(9.971965),
                new Rotation3d(Units.degreesToRadians(90.0), -Units.degreesToRadians(30), 0.0)));
    public static final Limelight CLIMBER =
        new Limelight(
            "limelight-climber",
            new Pose3d(
                -0.32076,
                -0.012695,
                0.168759,
                new Rotation3d(0, -Units.degreesToRadians(25.0), Math.PI)));
  }

  private PhotonCamera reefRight;
  private PhotonCamera reefLeft;
  private PhotonCamera frontElevator;
  private PhotonCamera climber;
  private PhotonCamera[] cameras;
  private PhotonCameraSim reefRightSim;
  private PhotonCameraSim reefLeftSim;
  private PhotonCameraSim frontElevatorSim;
  private PhotonCameraSim climberSim;
  private PhotonCameraSim[] simCameras;
  private VisionSystemSim visionSim;

  private final Drivetrain drivetrain;
  private final Limelight[] limelights =
      new Limelight[] {
        Limelights.REEF_RIGHT, Limelights.REEF_LEFT, Limelights.FRONT_ELEVATOR, Limelights.CLIMBER
      };

  //private final Notifier visionThread;

  public static boolean hasEnabled = false;

  public VisionManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    /* visionThread =
        new Notifier(
            () -> {
              for (Limelight ll : limelights) {
                addGlobalPoseEstimate(ll);
              }
              if (Robot.isSimulation() && visionSim != null) {
                visionSim.update(drivetrain.getEstimatedPose());
                for (int i = 0; i < 4; i++) {
                  for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
                    writeToTable(
                        result,
                        NetworkTableInstance.getDefault().getTable(limelights[i].name()),
                        GeomUtil.pose3dToTransform3d(limelights[i].relativePosition()).inverse());
                  }
                }
              }
            });
    visionThread.startPeriodic(Constants.PERIOD_SEC); */

    if (Robot.isSimulation()) {
      reefRight = new PhotonCamera("reefA");
      reefLeft = new PhotonCamera("reefB");
      frontElevator = new PhotonCamera("elevator");
      climber = new PhotonCamera("station");
      cameras = new PhotonCamera[] {reefRight, reefLeft, frontElevator, climber};

      visionSim = new VisionSystemSim("main");

      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

      var cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(
          VisionConstants.resolutionWidth,
          VisionConstants.resolutionHeight,
          Rotation2d.fromDegrees(91.145));
      cameraProps.setCalibError(0.35, 0.10);
      cameraProps.setFPS(15.0);
      cameraProps.setAvgLatencyMs(10);
      cameraProps.setLatencyStdDevMs(3);

      reefRightSim = new PhotonCameraSim(reefRight, cameraProps);
      reefLeftSim = new PhotonCameraSim(reefLeft, cameraProps);
      frontElevatorSim = new PhotonCameraSim(frontElevator, cameraProps);
      climberSim = new PhotonCameraSim(climber, cameraProps);

      simCameras = new PhotonCameraSim[] {reefRightSim, reefLeftSim, frontElevatorSim, climberSim};

      visionSim.addCamera(
          reefRightSim, GeomUtil.pose3dToTransform3d(Limelights.REEF_RIGHT.relativePosition()));
      visionSim.addCamera(
          reefLeftSim, GeomUtil.pose3dToTransform3d(Limelights.REEF_LEFT.relativePosition()));
      visionSim.addCamera(
          frontElevatorSim,
          GeomUtil.pose3dToTransform3d(Limelights.FRONT_ELEVATOR.relativePosition()));
      visionSim.addCamera(
          climberSim, GeomUtil.pose3dToTransform3d(Limelights.CLIMBER.relativePosition()));

      for (PhotonCameraSim camera : simCameras) {
        camera.enableRawStream(true);
        camera.enableProcessedStream(true);
        camera.enableDrawWireframe(false);
      }
    }
  }

  private void addGlobalPoseEstimate(Limelight limelight) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);
    PoseEstimate mtEstimate;
    PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
      
    if(!hasEnabled){
      mtEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());
    if (isValidEstimate(mtEstimate)) {
      //Logger.log("Vision/" + limelight.name() + "/MegaTagEstimate", mtEstimate.pose);
      if (!hasEnabled || Dashboard.disableMegatag2.get()) {
        double stds = 7.5;
        drivetrain.addVisionMeasurement(
            mtEstimate.pose, mtEstimate.timestampSeconds, VecBuilder.fill(stds, stds, stds * 2));
        Logger.log("Vision/" + limelight.name() + "/XyStds", stds);
      }
    }
  }

    if (isValidEstimate(mt2Estimate) && hasEnabled && !Dashboard.disableMegatag2.get()) {
      //Logger.log("Vision/" + limelight.name() + "/MegaTag2Estimate", mt2Estimate.pose);
      double xyStds = Math.pow(0.8, mt2Estimate.tagCount) * (mt2Estimate.avgTagDist * 3);
      drivetrain.addVisionMeasurement(
          mt2Estimate.pose, mt2Estimate.timestampSeconds, VecBuilder.fill(xyStds, xyStds, 999999999.999999));
      Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
    }
  }

  /*   private void addSpecializedPoseEstimate(Limelight limelight) {
    if (getReefZone().isEmpty()) {
      return;
    }
    Pair<Integer, Pose2d> desiredTag =
        AllianceFlipUtil.get(FieldConstants.BLUE_REEF_TAGS, FieldConstants.RED_REEF_TAGS)
            .get(getReefZone().getAsInt());
    Logger.log("Vision/DesiredTag", desiredTag.getFirst());
    RawFiducial[] results = LimelightHelpers.getRawFiducials(limelight.name());

    Arrays.stream(results)
        .filter(target -> target.id == desiredTag.getFirst())
        .forEach(
            target -> {
              double distance = target.distToRobot;
              Logger.log("Vision/" + limelight.name() + "/DistanceToTarget", distance);
              Rotation2d camToTag =
                  drivetrain
                      .getHeading()
                      .plus(
                          Rotation2d.fromDegrees(
                              Units.radiansToDegrees(
                                      limelight.relativePosition().getRotation().getZ())
                                  - target.txnc));
              Translation2d fieldToCamera =
                  new Pose2d(desiredTag.getSecond().getTranslation(), camToTag.plus(Rotation2d.kPi))
                      .transformBy(
                          GeomUtil.translationToTransform(new Translation2d(distance, 0.0)))
                      .getTranslation();
              Pose2d fieldToRobot =
                  new Pose2d(fieldToCamera, drivetrain.getHeading())
                      .transformBy(
                          new Transform2d(limelight.relativePosition().toPose2d(), Pose2d.kZero));
              drivetrain.specializedPoseEstimate.addVisionMeasurement(
                  new Pose2d(fieldToRobot.getTranslation(), drivetrain.getHeading()),
                  Timer.getFPGATimestamp() - (LimelightVision.getLatency(limelight) / 1000.0),
                  VecBuilder.fill(10.0, 10.0, 999999999));
            });
  } */

  private boolean isValidEstimate(PoseEstimate estimate) {
    return estimate != null
        && estimate.tagCount != 0
        && FieldConstants.FIELD_AREA.contains(estimate.pose)
        && !(Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 540)
        && !(drivetrain.getLinearVelocity().getNorm() > 3.0);
        //&& (DriverStation.isDisabled() || drivetrain.getEstimatedPose().getTranslation().getDistance(estimate.pose.getTranslation()) < 3.0);
  }

  public void periodic(){
    for (Limelight ll : limelights) {
      addGlobalPoseEstimate(ll);
    }
  }

  private void writeToTable(
      PhotonPipelineResult result, NetworkTable table, Transform3d cameraToRobot) {
    double latencyMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0;
    if (result.getMultiTagResult().isPresent()) {
      MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();
      Transform3d best = multiTagResult.estimatedPose.best.plus(cameraToRobot);
      Pose2d fieldToCamera =
          new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());

      int targetCount = result.targets.size();
      List<Double> pose_data = new ArrayList<>(11);
      List<Double> rawFiducial_data = new ArrayList<>(targetCount * 7);

      pose_data.addAll(
          Arrays.asList(
              best.getX(),
              best.getY(),
              best.getZ(),
              0.0, // roll
              0.0, // pitch
              fieldToCamera.getRotation().getDegrees(),
              latencyMs,
              (double) multiTagResult.fiducialIDsUsed.size(),
              0.0, // tag span
              calculateAverageTagDistance(result), // avg tag dist
              result.getBestTarget().getArea()));

      for (PhotonTrackedTarget target : result.targets) {
        rawFiducial_data.add((double) target.getFiducialId());
        rawFiducial_data.add(target.getYaw());
        rawFiducial_data.add(target.getPitch());
        rawFiducial_data.add(target.getArea()); // ta
        rawFiducial_data.add(
            target.getBestCameraToTarget().getTranslation().getNorm()); // distToCamera
        rawFiducial_data.add(
            target
                .getBestCameraToTarget()
                .plus(cameraToRobot)
                .getTranslation()
                .getNorm()); // distToRobot
        rawFiducial_data.add(target.getPoseAmbiguity()); // ambiguity
      }

      double[] poseArray = pose_data.stream().mapToDouble(Double::doubleValue).toArray();
      double[] rawFiducialArray =
          rawFiducial_data.stream().mapToDouble(Double::doubleValue).toArray();
      table.getEntry("rawfiducials").setDoubleArray(rawFiducialArray);
      table.getEntry("botpose_wpiblue").setDoubleArray(poseArray);
      table.getEntry("botpose_orb_wpiblue").setDoubleArray(poseArray);
    }

    table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
    table
        .getEntry("cl")
        .setDouble((Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0);
  }

  private double calculateAverageTagDistance(PhotonPipelineResult result) {
    double distance = 0;
    for (PhotonTrackedTarget target : result.targets) {
      distance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return distance / result.targets.size();
  }

  public OptionalInt getReefZone() {
    return AllianceFlipUtil.get(FieldConstants.BLUE_REEF, FieldConstants.RED_REEF)
        .contains(drivetrain.getEstimatedPose().getTranslation());
  }
}
