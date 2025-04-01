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
import edu.wpi.first.wpilibj.Timer;
import frc2025.Dashboard;
import frc2025.Robot;
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
                0.257199,
                -0.240415,
                0.197419,
                new Rotation3d(0, -Units.degreesToRadians(20.0), Units.degreesToRadians(35))));
    public static final Limelight STATION =
        new Limelight(
            "limelight-station",
            new Pose3d(
                -Units.inchesToMeters(1.492570),
                -Units.inchesToMeters(5.0),
                Units.inchesToMeters(40.156036),
                new Rotation3d(0.0, -Units.degreesToRadians(30), Math.PI)));
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

  private enum VisionType {
    MT,
    MT2;
  }

  private final Drivetrain drivetrain;
  private final Limelight[] limelights =
      new Limelight[] {
        Limelights.REEF_RIGHT, Limelights.REEF_LEFT // , Limelights.STATION, //Limelights.CLIMBER
      };

  // private final Notifier visionThread;

  public static boolean hasEnabled = false;

  public VisionManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    LimelightHelpers.SetFiducialIDFiltersOverride(
        Limelights.REEF_LEFT.name(), FieldConstants.REEF_TAGS);
    LimelightHelpers.SetFiducialIDFiltersOverride(
        Limelights.REEF_RIGHT.name(), FieldConstants.REEF_TAGS);

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
      climber = new PhotonCamera("climber");
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
          frontElevatorSim, GeomUtil.pose3dToTransform3d(Limelights.STATION.relativePosition()));
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
    // PoseEstimate mt2Estimate =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
    PoseEstimate mtEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name());

    boolean shouldUseMt1 = true; // !hasEnabled || Dashboard.disableMegatag2.get();
    /* boolean shouldUseMt2 =
    hasEnabled && !rejectEstimate(mt2Estimate) && !Dashboard.disableMegatag2.get(); */

    if (shouldUseMt1 && !Dashboard.disableAllVisionUpdates.get()) {
      if (!rejectEstimate(mtEstimate)) {
        double stdFactor = Math.pow(mtEstimate.avgTagDist, 2.75) / (mtEstimate.tagCount * 0.5);
        double xyStds =
            (DriverStation.isDisabled() ? 0.2 : VisionConstants.xyStdBaseline) * stdFactor;
        double thetaStds =
            (DriverStation.isDisabled() ? 0.2 : VisionConstants.thetaStdBaseline) * stdFactor;
        drivetrain.addVisionMeasurement(
            mtEstimate.pose,
            mtEstimate.timestampSeconds,
            VecBuilder.fill(xyStds, xyStds, thetaStds),
            false);
        Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT);
        Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
        Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
      } else {
        Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
        Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
        Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
      }
    }

    /* if (shouldUseMt2 && !Dashboard.disableAllVisionUpdates.get()) {
      double stdFactor = Math.pow(mt2Estimate.avgTagDist, 2.75) / (mt2Estimate.tagCount * 0.5);
      double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;
      double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;
      Pose2d combinedPose =
          new Pose2d(mt2Estimate.pose.getTranslation(), mtEstimate.pose.getRotation());
      drivetrain.addVisionMeasurement(
          mt2Estimate.pose,
          mt2Estimate.timestampSeconds,
          VecBuilder.fill(xyStds, xyStds, 999999999999.0),
          true);

        Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT2);
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", combinedPose, 1.5);
      Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
    } else {
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
      Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
    } */
  }

  private boolean rejectEstimate(PoseEstimate estimate) {
    return estimate == null
        || estimate.tagCount == 0
        || !FieldConstants.FIELD_AREA.contains(estimate.pose)
        || (estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.3)
        || (Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 540)
        || (drivetrain.getLinearVelocity().getNorm() > 3.5);
  }

  public void periodic() {
    for (Limelight ll : limelights) {
      addGlobalPoseEstimate(ll);
    }

    if (Robot.isSimulation() && visionSim != null) {
      visionSim.update(drivetrain.getEstimatedPose());
      for (int i = 0; i < limelights.length; i++) {
        for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
          writeToTable(
              result,
              NetworkTableInstance.getDefault().getTable(limelights[i].name()),
              GeomUtil.pose3dToTransform3d(limelights[i].relativePosition()).inverse());
        }
      }
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
