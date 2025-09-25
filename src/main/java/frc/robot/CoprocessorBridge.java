package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.RosMessage;
import frc.team88.ros.messages.geometry_msgs.Pose2D;
import frc.team88.ros.messages.std_msgs.RosFloat64;
import org.photonvision.*;

/**
 * CoprocessorBridge is an example class demonstrating how to use the
 * ROSNetworkTablesBridge to enable communication between a WPILib Java robot
 * project and a ROS environment.
 */
public class CoprocessorBridge extends SubsystemBase {
    private final ROSNetworkTablesBridge m_ros_interface;
    private final BridgeSubscriber<RosFloat64> m_pingSendSub;
    private final BridgePublisher<RosFloat64> m_pingReturnPub;
    private final BridgeSubscriber<RosFloat64> m_odomSubPosePositionXComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPosePositionYComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPosePositionZComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPoseOrientationXComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPoseOrientationYComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPoseOrientationZComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubPoseOrientationWComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistAngularXComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistAngularYComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistAngularZComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistLinearXComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistLinearYComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTwistLinearZComponent;
    private final BridgeSubscriber<RosFloat64> m_odomSubTimestampSeconds;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagPositionXComponent;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagPositionYComponent;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagPositionZComponent;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagOrientationXComponent;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagOrientationYComponent;
    private final BridgePublisher<RosFloat64> m_odomPubApriltagOrientationZComponent;

    private PhotonPoseEstimator initalPoseEstimator;
    private PhotonCamera camera;
    
    // do not access directly
    private Pose3d _INTERNALS_OUT_;

    /**
     * Constructor for the CoprocessorBridge class.
     */
    public CoprocessorBridge(PhotonPoseEstimator estimator, PhotonCamera camera) {
        long updateDelay = 20;
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        m_ros_interface = new ROSNetworkTablesBridge(instance.getTable(""), updateDelay);

        // Initialize ROS bridge objects
        m_pingSendSub = new BridgeSubscriber<>(m_ros_interface, "/ping_send", RosFloat64.class);
        m_pingReturnPub = new BridgePublisher<>(m_ros_interface, "/ping_return");

        m_odomSubPosePositionXComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/position/x", RosFloat64.class);
        m_odomSubPosePositionYComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/position/y", RosFloat64.class);
        m_odomSubPosePositionZComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/position/z", RosFloat64.class);
        m_odomSubPoseOrientationXComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/orientation/x", RosFloat64.class);
        m_odomSubPoseOrientationYComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/orientation/y", RosFloat64.class);
        m_odomSubPoseOrientationZComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/orientation/z", RosFloat64.class);
        m_odomSubPoseOrientationWComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/pose/orientation/w", RosFloat64.class);
        m_odomSubTwistAngularXComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/twist/angular/x", RosFloat64.class);
        m_odomSubTwistAngularYComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/twist/angular/y", RosFloat64.class);
        m_odomSubTwistAngularZComponent = new BridgeSubscriber<>(m_ros_interface,
                "/odom_decomposer/odom/twist/angular/z", RosFloat64.class);
        m_odomSubTwistLinearXComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/x",
                RosFloat64.class);
        m_odomSubTwistLinearYComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/y",
                RosFloat64.class);
        m_odomSubTwistLinearZComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/z",
                RosFloat64.class);
        m_odomSubTimestampSeconds = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/header/stamp/sec",
                RosFloat64.class);

        m_odomPubApriltagPositionXComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_position_x");
        m_odomPubApriltagPositionYComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_position_y");
        m_odomPubApriltagPositionZComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_position_z");
        m_odomPubApriltagOrientationXComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_orientation_x");
        m_odomPubApriltagOrientationYComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_orientation_y");
        m_odomPubApriltagOrientationZComponent = new BridgePublisher<>(m_ros_interface, "/inital_pose_orientation_z");

        this.initalPoseEstimator = estimator;
        this.camera = camera;
    }

    /**
     * Checks if a new ping message has been received from the ROS environment,
     * and if so, sends it back as a response. This measures the round trip response
     * time.
     */
    public void checkPing() {
        Optional<RosFloat64> ping;
        if ((ping = m_pingSendSub.receive()).isPresent()) {
            m_pingReturnPub.send(ping.get());
        }
    }

    /**
     * Gets 2d Rotation from latest pose update
     * 
     * @return Yaw of the robot, as a double
     */
    public double getRotation2d() {
        Optional<RosFloat64> z = m_odomSubPoseOrientationZComponent.receive();
        Optional<RosFloat64> w = m_odomSubPoseOrientationWComponent.receive();

        if (z.isPresent() && w.isPresent()) {
            // Convert quaternion to yaw angle (2D rotation)
            return 2.0 * Math.atan2(z.get().getData(), w.get().getData());
        }
        return 0.0; // Default value if no data available
    }

    /**
     * Gets 2d Pose from latest pose update
     * 
     * @return Pose2D
     */
    public Pose2d getPose2d() {
        Optional<RosFloat64> x = m_odomSubPosePositionXComponent.receive();
        Optional<RosFloat64> y = m_odomSubPosePositionYComponent.receive();

        if (x.isPresent() && y.isPresent()) {
            return new Pose2d(x.get().getData(), y.get().getData(), new Rotation2d(getRotation2d()));
        }
        return new Pose2d(); // Default pose if no data available
    }

    /**
     * Gets timestamp from latest pose update
     * 
     * @return The timestamp in seconds as a double
     */
    public double getTimestamp() {
        return m_odomSubTimestampSeconds.receive().get().getData();
    }

    /**
     * Gets pose from vision subsytem and sends it to ROS to initalize localization.
     * MUST BE CALLED!
     * 
     * @return nothing
     */
    public void sendInitalPose() {
        double robotPoseX;
        double robotPoseY;
        double robotPoseZ;
        double robotOrientationX;
        double robotOrientationY;
        double robotOrientationZ;
        Pose3d robotPose3d;
        robotPose3d = getPose3dFromPhotonvision();
        robotPoseX = robotPose3d.getX();
        robotPoseY = robotPose3d.getY();
        robotPoseZ = robotPose3d.getZ();
        robotOrientationX = robotPose3d.getRotation().getX();
        robotOrientationY = robotPose3d.getRotation().getY();
        robotOrientationZ = robotPose3d.getRotation().getZ();

        m_odomPubApriltagPositionXComponent.send(new RosFloat64(robotPoseX));
        m_odomPubApriltagPositionYComponent.send(new RosFloat64(robotPoseY));
        m_odomPubApriltagPositionZComponent.send(new RosFloat64(robotPoseZ));
        m_odomPubApriltagOrientationXComponent.send(new RosFloat64(robotOrientationX));
        m_odomPubApriltagOrientationYComponent.send(new RosFloat64(robotOrientationY));
        m_odomPubApriltagOrientationZComponent.send(new RosFloat64(robotOrientationZ));
    }

    /**
     * Gets Pose3d of the robot from Photonvision, used to initialize the robot's position
     * @return Pose3d of the robot
     */
    public Pose3d getPose3dFromPhotonvision() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = initalPoseEstimator.update(change);
            visionEst.ifPresent(
                    est -> {
                        //estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        _INTERNALS_OUT_ = new Pose3d(est.estimatedPose.getX(), est.estimatedPose.getY(), est.estimatedPose.getZ(), est.estimatedPose.getRotation());
                    });
        }
        return _INTERNALS_OUT_;
    }
}