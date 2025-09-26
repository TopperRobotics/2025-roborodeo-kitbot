package frc.robot.vision;

import java.util.Optional;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.*;
import frc.robot.Constants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//import com.google.flatbuffers.Constants;

public class localizeRobot {
    private final SwerveSubsystem s_Swerve;
    private Alliance alliance;
    private AprilTagFieldLayout aprilTagFieldLayout;
    
    // Multiple cameras
    private PhotonCamera frontCamera;
    private PhotonCamera backCamera;
    // Add more cameras as needed
    
    // Pose estimators for each camera
    private PhotonPoseEstimator frontPoseEstimator;
    private PhotonPoseEstimator backPoseEstimator;
    
    // Camera transforms (robot-to-camera)
    private Transform3d robotToFrontCam;
    private Transform3d robotToBackCam;

    // do not access directly
    private Pose3d _INTERNALS_OUT_;

    public localizeRobot(SwerveSubsystem s_Swerve) {
        this.s_Swerve = s_Swerve;
        
        try {
            // Load the AprilTag field layout
            aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
            return;
        }
        
        // Initialize cameras
        frontCamera = new PhotonCamera("cirno");
        backCamera = new PhotonCamera("reimu");
        
        // Define camera positions relative to robot center
        // Example transforms - adjust these to match your robot's camera mounting
        robotToFrontCam = Constants.VisionConstants.robotToFrontCameraTransform;
        
        robotToBackCam = Constants.VisionConstants.robotToBackCameraTransform;
        
        // Create pose estimators for each camera
        frontPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToFrontCam
        );
        
        backPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackCam
        );
        
        // Set multi-tag fallback strategy
        frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateOdomWithMultiCamera() {
        // Get current alliance
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            System.out.println("Alliance not found");
            return;
        }
        alliance = allianceOptional.get();
        
        // Process estimates from all cameras
        processCamera(frontPoseEstimator, frontCamera, "cirno");
        processCamera(backPoseEstimator, backCamera, "reimu");
    }
    
    private void processCamera(PhotonPoseEstimator poseEstimator, PhotonCamera camera, String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            System.out.println("No targets found on " + cameraName + " camera");
            return;
        }
        
        // Update pose estimator with latest result
        poseEstimator.update(result);
        
        // Get pose estimate
        Optional<EstimatedRobotPose> poseOpt = poseEstimator.update(result);
        
        if (poseOpt.isEmpty()) {
            System.out.println("No pose estimate from " + cameraName + " camera");
            return;
        }
        
        EstimatedRobotPose estimatedPose = poseOpt.get();
        Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
        double timestamp = estimatedPose.timestampSeconds;
        
        // Filter pose estimates based on distance and ambiguity
        if (shouldAcceptPoseEstimate(result, estimatedPose, cameraName)) {
            s_Swerve.addVisionMeasurement(pose2d, timestamp);
            
            // Log information about detected targets
            logTargetInfo(result, cameraName, pose2d);
        }
    }
    
    private boolean shouldAcceptPoseEstimate(PhotonPipelineResult result, 
                                           EstimatedRobotPose estimatedPose, 
                                           String cameraName) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        
        // Reject if no targets
        if (targets.isEmpty()) {
            return false;
        }
        
        // Check distance to closest target
        double minDistance = targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.MAX_VALUE);
        
        // Reject if targets are too far (adjust threshold as needed)
        if (minDistance > 8.0) { // 8 meters
            System.out.println("Rejecting " + cameraName + " estimate: targets too far (" + 
                             String.format("%.2f", minDistance) + "m)");
            return false;
        }
        
        // Check ambiguity for single-tag estimates
        if (targets.size() == 1) {
            double ambiguity = targets.get(0).getPoseAmbiguity();
            if (ambiguity > 0.2) { // Adjust threshold as needed
                System.out.println("Rejecting " + cameraName + " estimate: high ambiguity (" + 
                                 String.format("%.3f", ambiguity) + ")");
                return false;
            }
        }
        
        // Additional filtering can be added here:
        // - Check if pose is within field boundaries
        // - Compare with odometry pose for reasonableness
        // - Check target area/quality metrics
        
        return true;
    }
    
    private void logTargetInfo(PhotonPipelineResult result, String cameraName, Pose2d robotPose) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        
        System.out.println("=== " + cameraName.toUpperCase() + " CAMERA ===");
        System.out.println("Robot pose: " + robotPose);
        System.out.println("Alliance: " + alliance);
        System.out.println("Targets detected: " + targets.size());
        
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            
            System.out.println("  Target " + (i + 1) + ":");
            System.out.println("    AprilTag ID: " + target.getFiducialId());
            System.out.println("    Distance: " + String.format("%.2f", distance) + "m");
            System.out.println("    Ambiguity: " + String.format("%.3f", target.getPoseAmbiguity()));
            System.out.println("    Area: " + String.format("%.1f", target.getArea()) + "%");
        }
    }
    
    public void initialPoseUpdate() {
        // Get current alliance
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            System.out.println("Alliance not found for initial pose update");
            return;
        }
        alliance = allianceOptional.get();
        
        System.out.println("Attempting initial pose update for " + alliance + " alliance");
        
        // Try to get initial pose from front camera first
        Optional<Pose2d> initialPose = getInitialPoseFromCamera(frontPoseEstimator, frontCamera, "front");
        
        // If front camera fails, try back camera
        if (initialPose.isEmpty()) {
            initialPose = getInitialPoseFromCamera(backPoseEstimator, backCamera, "back");
        }
        
        if (initialPose.isPresent()) {
            s_Swerve.resetOdometry(initialPose.get());
            System.out.println("Initial pose set to: " + initialPose.get());
        } else {
            System.out.println("Failed to get initial pose from any camera");
        }
    }
    
    private Optional<Pose2d> getInitialPoseFromCamera(PhotonPoseEstimator poseEstimator, 
                                                     PhotonCamera camera, 
                                                     String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            System.out.println("No targets found on " + cameraName + " camera for initial pose");
            return Optional.empty();
        }
        
        Optional<EstimatedRobotPose> poseOpt = poseEstimator.update(result);
        
        if (poseOpt.isEmpty()) {
            System.out.println("No pose estimate from " + cameraName + " camera for initial pose");
            return Optional.empty();
        }
        
        EstimatedRobotPose estimatedPose = poseOpt.get();
        
        if (shouldAcceptPoseEstimate(result, estimatedPose, cameraName)) {
            System.out.println("Initial pose obtained from " + cameraName + " camera");
            logTargetInfo(result, cameraName, estimatedPose.estimatedPose.toPose2d());
            return Optional.of(estimatedPose.estimatedPose.toPose2d());
        }
        
        return Optional.empty();
    }

    /**
     * Gets Pose3d of the robot from Photonvision, used to initialize the robot's position
     * @return Pose3d of the robot
     */
    public Pose3d getPose3dFromPhotonvision() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : frontCamera.getAllUnreadResults()) {
            visionEst = frontPoseEstimator.update(change);
            visionEst.ifPresent(
                    est -> {
                        //estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        _INTERNALS_OUT_ = new Pose3d(est.estimatedPose.getX(), est.estimatedPose.getY(), est.estimatedPose.getZ(), est.estimatedPose.getRotation());
                    });
        }
        return _INTERNALS_OUT_;
    }

    public PhotonCamera getFrontPhotonCamera(){
        return frontCamera;
    }

    public PhotonCamera getBackPhotonCamera(){
        return backCamera;
    }
}