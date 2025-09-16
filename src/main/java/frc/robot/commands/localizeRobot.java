package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class localizeRobot {
    private final SwerveSubsystem s_Swerve;
    private Alliance alliance = DriverStation.getAlliance().get();
    private PoseEstimate estimatedPose;
    private Pose2d pose = estimatedPose.pose;
    private double poseTimestamp = estimatedPose.timestampSeconds;

    public localizeRobot(SwerveSubsystem s_Swerve){
        this.s_Swerve = s_Swerve;
        if(alliance.equals(Alliance.Blue)){
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        } else if(alliance.equals(Alliance.Red)){
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        } else {
            estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        }
    }

    public void updateOdomWithMT2(){
        s_Swerve.addVisionMeasurement(pose, poseTimestamp);
    }
}
