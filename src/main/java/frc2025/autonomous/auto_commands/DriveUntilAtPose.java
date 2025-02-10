package frc2025.autonomous.auto_commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc2025.RobotContainer;
import frc2025.commands.DriveToPose;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;

public class DriveUntilAtPose extends DriveToPose{

    private final Drivetrain drivetrain;
    private final Pose2d targetPose;
    
    public DriveUntilAtPose(Pose2d pose, RobotContainer container){
        super(container.getSubsystems().drivetrain(), pose);
        drivetrain = container.getSubsystems().drivetrain();
        targetPose = pose;
        addRequirements(drivetrain);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) <= DrivetrainConstants.AT_POSE_DIST_THRESHOLD.getMeters();
    }
}
