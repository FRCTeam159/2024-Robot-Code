// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TagDetector;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_Controller = new XboxController(0);
  //Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain();
  //private final Autonomous m_auto = new Autonomous(m_Drivetrain);
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_Drivetrain, m_Controller);
  private final TagDetector m_detector= new TagDetector(m_Drivetrain);
  
  //commands
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drivetrain.setDefaultCommand(m_DriveWithGamepad);
    // Configure the button bindings
    configureBindings();
     AutoBuilder.configureHolonomic(
      m_Drivetrain::getPose, // Robot pose supplier
      m_Drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      m_Drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      m_Drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        m_Drivetrain.kMaxVelocity, // Max module speed, in m/s
        0.3, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        /*
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        */
        return false;
      },
      m_Drivetrain // Reference to this subsystem to set requirements
    );
  }
  public void robotInit() {
    m_detector.start();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {}

  public void autonomousInit() {
    m_Drivetrain.reset();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //m_Drivetrain.resetPose(new Pose2d());
    return AutoBuilder.buildAuto("CircleAuto");
  }
}
