package frc.robot.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.*;
import com.pathplanner.*;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

public class moveAndRotate extends Command {
    private final SwerveSubsystem s_Swerve;
    private final PIDController moveXController = new PIDController(2.1, 0, 0);
    private final PIDController moveYController = new PIDController(2.1, 0, 0);
    private final PIDController moveTController = new PIDController(5, 0, 0);
    private final localizeRobot localizer;
    private PhotonPipelineResult pipelineResult;

    private final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private boolean isDone;
    private Pose2d targetPose;
    private boolean isBlue;

    public moveAndRotate(SwerveSubsystem s_Swerve, localizeRobot s_Localize) {
        this.s_Swerve = s_Swerve;
        this.localizer = s_Localize;

        camera = new PhotonCamera("cirno");

        poseEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, // helps disambiguate
            //camera,
            Constants.VisionConstants.robotToFrontCameraTransform
        );

        addRequirements(s_Swerve);
    }

    private double optimizeAngle(Rotation2d currentAngle, Rotation2d targetAngle) {
        double diff = targetAngle.minus(currentAngle).getDegrees();
        if (Math.abs(diff) > 180) {
            return (diff < 0) ? diff + 360 : diff - 360;
        }
        return diff;
    }

    private boolean isPoseReasonable(Pose2d estPose, Pose2d odoPose) {
        double dist = estPose.getTranslation().getDistance(odoPose.getTranslation());
        double rotDiff = Math.abs(optimizeAngle(odoPose.getRotation(), estPose.getRotation()));

        // Reject if too far (>3m) or rotated >90°
        return dist < 3.0 && rotDiff < 90.0;
    }

    private PhotonPipelineResult getBestResult(List<PhotonPipelineResult> results) {
        if (results.isEmpty()) {
            return null;
        }

        // Return best result (highest target area)
        PhotonPipelineResult best = results.get(0);
        for (PhotonPipelineResult r : results) {
            if (r.getBestTarget().getArea() > best.getBestTarget().getArea()) {
                best = r;
            }
        }
        return best;
    }

    @Override
    public void initialize() {
        isDone = false;

        isBlue = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Blue;

        // Seed with odometry pose
        poseEstimator.setReferencePose(s_Swerve.getPose());
        
        pipelineResult = camera.getLatestResult();
        if (pipelineResult.hasTargets()) {
            System.out.println("No targets found");
            isDone = true;
            return;
        }

        Optional<EstimatedRobotPose> estOpt = poseEstimator.update(pipelineResult);

        if (estOpt.isEmpty()) {
            System.out.println("No estimated pose available");
            isDone = true;
            return;
        }

        EstimatedRobotPose est = estOpt.get();
        Pose2d estPose = est.estimatedPose.toPose2d();

        // Reject bad estimates
        if (!isPoseReasonable(estPose, s_Swerve.getPose())) {
            System.out.println("Rejected unreasonable estimated pose: " + estPose);
            isDone = true;
            return;
        }

        targetPose = estPose;
        System.out.println("Using estimated target pose: " + targetPose);
    }

    @Override
    public void execute() {
        if (isDone) {
            return;
        }

        Pose2d robotPose = s_Swerve.getPose();

        double delx = targetPose.getX() - robotPose.getX();
        double dely = targetPose.getY() - robotPose.getY();
        double delt = optimizeAngle(robotPose.getRotation(), targetPose.getRotation());

        System.out.printf("Diffs -> x: %.2f, y: %.2f, t: %.2f%n", delx, dely, delt);

        double xOutput = Math.min(moveXController.calculate(isBlue ? -delx : delx), 3);
        double yOutput = Math.min(moveYController.calculate(isBlue ? -dely : dely), 3);
        double tOutput = Math.min(moveTController.calculate(-delt), 3);

        s_Swerve.drive(new Translation2d(xOutput, yOutput), tOutput, true);

        if (Math.abs(delx) < 0.1 && Math.abs(dely) < 0.1 && Math.abs(delt) < 2) {
            isDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DONE");
        s_Swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
