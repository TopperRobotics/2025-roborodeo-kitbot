// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.scorer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.vision.localizeRobot;
import frc.robot.vision.moveAndRotate;

import java.io.File;
import swervelib.SwerveInputStream;
import java.util.Timer;

import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);

  final        CommandJoystick simDriver = new CommandJoystick(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  scorer scoringMotor0 = new scorer();
  localizeRobot robotLocalizer = new localizeRobot(drivebase);
  moveAndRotate MR_Tag = new moveAndRotate(drivebase, robotLocalizer);
  VisionSystemSim visionSim = new VisionSystemSim("main");
  //private final CoprocessorBridge m_bridge = new CoprocessorBridge();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Converts joystick driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocityJoystick = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> simDriver.getY() * -1,
                                                                () -> simDriver.getX() * -1)
                                                            .withControllerRotationAxis(simDriver::getTwist)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngleJoystick = driveAngularVelocityJoystick.copy()//.withControllerHeadingAxis(simDriver::getTwist)
                                                           .headingWhile(false)
                                                           .translationHeadingOffset(true)
                                                           .translationHeadingOffset(Rotation2d.fromDegrees(45));

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
/*
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  simDriver.getY() *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  simDriver.getX() *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   45));*/

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    // localize robot on enable
    //robotLocalizer.updateOdomWithMT2();
    //NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // twin this should not be here 🥀
    // centers modules on enable
    //drivebase.centerModulesNonCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    drivebase.zeroGyro();

    //drivebase.setModulesToIMUYaw();
    drivebase.printModuleAngles();
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleJoystick);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityJoystick);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleJoystick);

    //if (RobotBase.isSimulation())
    //{
    //  drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    //} else
    //{
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //}
////
    /*if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleJoystick.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleJoystick.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleJoystick.driveToPoseEnabled(false)));
      simDriver.button(2).onTrue(Commands.runOnce(drivebase::zeroGyro).andThen(drivebase.centerModulesCommand()));



//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    /*if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }*/
    driverXbox.b().whileTrue(drivebase.centerModulesCommand());
    driverXbox.b().onFalse(Commands.none());
    driverXbox.a().whileTrue(Commands.runOnce(drivebase::zeroGyro).withTimeout(0.2).andThen(drivebase.centerModulesCommand()));
    driverXbox.a().onFalse(Commands.none());
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // driverXbox.povUp().onTrue(drivebase.setModuleToAngle(0, 90));
    // driverXbox.povRight().onTrue(drivebase.setModuleToAngle(1, 90));
    // driverXbox.povDown().onTrue(drivebase.setModuleToAngle(2, 90));
    // driverXbox.povLeft().onTrue(drivebase.setModuleToAngle(3, 90));

    driverXbox.y().whileTrue(MR_Tag);
    driverXbox.y().onFalse(Commands.none());
    //driverXbox.y().onFalse(Commands.runOnce(()->MR_Tag.end(true)));
    //scorerXbox.a().onFalse(Commands.runOnce(() -> scoringMotor0.stopMotor()));
    
    driverXbox.rightTrigger().whileTrue(Commands.runOnce(() -> scoringMotor0.runMotorBackwards()));
    driverXbox.rightTrigger().onFalse(Commands.runOnce(() -> scoringMotor0.stopMotor()));

    driverXbox.leftTrigger().onTrue(Commands.runOnce(() -> robotLocalizer.updateOdomWithMultiCamera()));
    driverXbox.leftTrigger().onFalse(Commands.none());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //TODO: finish this!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //public Command getAutonomousCommand()
  //{
    // An example command will be run in autonomous
  //  return drivebase.getAutonomousCommand("New Auto");
    /*return new InstantCommand(()->{
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)){
        drivebase.setGyro(180);
      } 
      else{ 
        drivebase.zeroGyro();
      }}).andThen(autoChooser.getSelected());//new PathPlannerAuto("MAuto"));
  *///}

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
