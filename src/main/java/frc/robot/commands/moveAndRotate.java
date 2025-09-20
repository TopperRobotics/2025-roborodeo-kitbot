package frc.robot.commands;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.*;

// migrate from custom swerve code to yagsl library
// also remove elevator code because we don't need it right now
public class moveAndRotate extends Command {
    private final SwerveSubsystem s_Swerve;
    private final PIDController moveXController = new PIDController(2.1, 0, 0);//(2.1, 0, 0);
    private final PIDController moveYController = new PIDController(2.1, 0, 0);//(2.1, 0, 0);
    private final PIDController moveTController = new PIDController(5, 0, 0);//(2.1, 0, 0);
    private final localizeRobot localizer;

    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private boolean isDone;
    private double x;
    private double y;
    private double thetaGoal;
    private boolean isBlue;
    private long curr_tag_in_view;
    public moveAndRotate(SwerveSubsystem s_Swerve, localizeRobot s_Localize) {
        this.s_Swerve= s_Swerve;
        this.localizer = s_Localize;
        // this.amount_offset = amount_offset;
        // moveXController.setTolerance(0.05);
        // moveYController.setTolerance(0.05);
        addRequirements(s_Swerve);
    }

    public double optimizeAngle(Rotation2d currentAngle, Rotation2d targetAngle){
        if(Math.abs(targetAngle.minus(currentAngle).getDegrees()) > 180){
          if((targetAngle.minus(currentAngle).getDegrees()) < 0){
            return ((targetAngle.minus(currentAngle).getDegrees()) + 360);
          }
          else{
            return ((targetAngle.minus(currentAngle).getDegrees()) - 360);
          }
        }else{
          return targetAngle.minus(currentAngle).getDegrees();
        }
    }
    
    public double getDistanceToTagInFeet(){
        Pose3d tag3dPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        return tag3dPose.getTranslation().getNorm() * 3.28084;
    }
    
    @Override
    public void initialize() {
        isDone = false;
        //s_Swerve.togglePreciseTargeting(true);
        curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)){
            isBlue = true;
        }
        else{
            isBlue = false;
        }
        if (curr_tag_in_view < 0){
            isDone = true;
            System.out.println("No apriltag");
        }
        else{
            var roboPose = s_Swerve.getPose();
            Pose2d tag_pose = new Pose2d();
            if (curr_tag_in_view > 0){
                tag_pose = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d();

            }
            // check to see if detected tag is further than 2 feet away
            if(getDistanceToTagInFeet()>4){
                isDone = true;
                System.out.println("further than 2 feet from tag");
                return;
            }
            double tag_x = tag_pose.getX();
            double tag_y = tag_pose.getY();
            Rotation2d tag_theta = tag_pose.getRotation().minus(Rotation2d.fromDegrees(90));
            Rotation2d tag_theta_rot = tag_pose.getRotation().plus(Rotation2d.fromDegrees(180));
            double diff_t = roboPose.getRotation().minus(tag_theta_rot).getDegrees();
            System.out.println("t_dff" + diff_t + "robot angle: " + diff_t);
            double forward_offset = -0.34;
            //String level = NetworkTableInstance.getDefault().getTable("ButtonBoard").getEntry("cumber").getString("");
            /*double o = 0; // o is lateral offset
            if (level.equals("R3") || level.equals("R2") || level.equals("R1")){
                o = 0.18;
                if(!isBlue){
                    o = .14;
                }
            }
            else if (level.equals("L3") || level.equals("L2") || level.equals("L1")){
                o = -0.3;
                if(!isBlue){
                    o = -0.3;//5;
                }

                
            }
            else{
                o = 0;
            }*/
            double lateral_offset = 0; //o;
            double x_offset = tag_x + forward_offset * tag_theta.getSin() - lateral_offset * tag_theta.getCos();
            double y_offset = tag_y - forward_offset * tag_theta.getCos() - lateral_offset * tag_theta.getSin();
            double diff_x = roboPose.getX() - tag_x;
            double diff_y = roboPose.getY() - tag_y;
            System.out.println("x_diff"+ diff_x + "y_dff" + diff_y + "angle" + roboPose.getRotation().minus(tag_theta_rot).getDegrees());
            x = x_offset;
            y = y_offset;
            thetaGoal = tag_theta_rot.getDegrees();
        }
        
    }
  
    @Override
    public void execute() {
        System.out.println("Executed main loop: "+ isDone);
        var robotPose2d = s_Swerve.getPose();
        if (isDone){
            
            return;
        }
        double delx = (x - robotPose2d.getX());
        double dely = (y - robotPose2d.getY());
        double delt = (optimizeAngle(Rotation2d.fromDegrees(robotPose2d.getRotation().getDegrees()), Rotation2d.fromDegrees(thetaGoal)));
        System.out.println("ID: "+ curr_tag_in_view + " diff x: " + delx + " diff y: " + dely + "diff t: " + delt);
        System.out.println("current location field relative: " + s_Swerve.getPose());
        // Output Volts is capped at 2 to prevent brownout
        double xOutput = Math.min(moveXController.calculate(isBlue? -1 * delx : delx), 3);
        double yOutput = Math.min(moveYController.calculate(isBlue? -1 * dely : dely), 3);
        double tOutput = Math.min(moveTController.calculate(-1*delt), 3);
        s_Swerve.drive(new Translation2d(xOutput, yOutput), tOutput, true);
        if (Math.abs(delx) < 0.1 && Math.abs(dely) < 0.1 && Math.abs(delt)<2 ){
            isDone = true;
        }
        else{
            isDone = false;
        }
        
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("DONE");
        // old command that was from the old custom library s_Swerve.stop();
        s_Swerve.drive(new Translation2d(0, 0), 0, true);
    }
    
}
