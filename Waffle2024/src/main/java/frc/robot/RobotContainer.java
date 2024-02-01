// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
//import frc.robot.subsystems.Camera;
//import frc.robot.subsystems.DetectorAprilTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.TargetMgr;
import edu.wpi.first.wpilibj2.command.Command;

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
  private final Autonomous m_auto = new Autonomous(m_Drivetrain);
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_Drivetrain, m_Controller);
  private final Arm m_Arm = new Arm();
  private final TagDetector m_TagDetector = new TagDetector(m_Drivetrain);

  //private final Camera m_Camera = new Camera();
  public final Limelight m_Limelight = new Limelight();

  //commands
  private final ShootNote m_shootNote = new ShootNote();

  //private final DetectorAprilTag m_apriltag = new DetectorAprilTag(m_Camera);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drivetrain.setDefaultCommand(m_DriveWithGamepad);
    
    // Commands
    //NamedCommands.registerCommand("shootNote", m_shootNote);
    
    // Configure the button bindings
    configureBindings();
  }
  public void robotInit() {
    m_TagDetector.start();
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
    m_Drivetrain.resetOdometry();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    m_Drivetrain.resetPose(new Pose2d());
    // Load the path you want to follow using its name in the GUI
    //PathPlannerPath path = PathPlannerPath.fromPathFile("ForwardPath");
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    //return AutoBuilder.followPath(path);
    //return m_auto.getCommand();

    return AutoBuilder.buildAuto("BlueCenter");
  }
  
  public void teleopInit() {
    m_Drivetrain.resetOdometry();
  }
}