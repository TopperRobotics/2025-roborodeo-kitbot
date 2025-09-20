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

    /**
     * Constructor for the CoprocessorBridge class.
     */
    public CoprocessorBridge() {
        long updateDelay = 20;
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        m_ros_interface = new ROSNetworkTablesBridge(instance.getTable(""), updateDelay);

        // Initialize ROS bridge objects
        m_pingSendSub = new BridgeSubscriber<>(m_ros_interface, "/ping_send", RosFloat64.class);
        m_pingReturnPub = new BridgePublisher<>(m_ros_interface, "/ping_return");

        m_odomSubPosePositionXComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/position/x", RosFloat64.class);
        m_odomSubPosePositionYComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/position/y", RosFloat64.class);
        m_odomSubPosePositionZComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/position/z", RosFloat64.class);
        m_odomSubPoseOrientationXComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/orientation/x", RosFloat64.class);
        m_odomSubPoseOrientationYComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/orientation/y", RosFloat64.class);
        m_odomSubPoseOrientationZComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/orientation/z", RosFloat64.class);
        m_odomSubPoseOrientationWComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/pose/orientation/w", RosFloat64.class);
        m_odomSubTwistAngularXComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/angular/x", RosFloat64.class);
        m_odomSubTwistAngularYComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/angular/y", RosFloat64.class);
        m_odomSubTwistAngularZComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/angular/z", RosFloat64.class);
        m_odomSubTwistLinearXComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/x", RosFloat64.class);
        m_odomSubTwistLinearYComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/y", RosFloat64.class);
        m_odomSubTwistLinearZComponent = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/twist/linear/z", RosFloat64.class);
        m_odomSubTimestampSeconds = new BridgeSubscriber<>(m_ros_interface, "/odom_decomposer/odom/header/stamp/sec", RosFloat64.class);
    }

    /**
     * Checks if a new ping message has been received from the ROS environment,
     * and if so, sends it back as a response. This measures the round trip response time.
     */
    public void checkPing() {
        Optional<RosFloat64> ping;
        if ((ping = m_pingSendSub.receive()).isPresent()) {
            m_pingReturnPub.send(ping.get());
        }
    }

    public double getRotation2d() {
        Optional<RosFloat64> z = m_odomSubPoseOrientationZComponent.receive();
        Optional<RosFloat64> w = m_odomSubPoseOrientationWComponent.receive();
        
        if (z.isPresent() && w.isPresent()) {
            // Convert quaternion to yaw angle (2D rotation)
            return 2.0 * Math.atan2(z.get().getData(), w.get().getData());
        }
        return 0.0; // Default value if no data available
    }
    
    public Pose2d getPose2d() {
        Optional<RosFloat64> x = m_odomSubPosePositionXComponent.receive();
        Optional<RosFloat64> y = m_odomSubPosePositionYComponent.receive();
        
        if (x.isPresent() && y.isPresent()) {
            return new Pose2d(x.get().getData(), y.get().getData(), new Rotation2d(getRotation2d()));
        }
        return new Pose2d(); // Default pose if no data available
    }

    public double getTimestamp(){
        return m_odomSubTimestampSeconds.receive().get().getData();
    }
}