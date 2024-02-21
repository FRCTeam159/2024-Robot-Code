// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmControls;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.IntakeShooterControls;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeShooter;
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
  private final Arm m_Arm = new Arm();
  //private final Autonomous m_auto = new Autonomous(m_Drivetrain);
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_Drivetrain, m_Controller);
  private final TagDetector m_detector = new TagDetector(m_Drivetrain);
  private final ArmControls m_ArmControls = new ArmControls(m_Arm, m_Controller);
  private final IntakeShooter m_IntakeShooter = new IntakeShooter();
  private final IntakeShooterControls m_IntakeShooterControls = new IntakeShooterControls(m_IntakeShooter, m_Arm, m_Controller);
  
  //commands
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Drivetrain.setDefaultCommand(m_DriveWithGamepad);
    m_Arm.setDefaultCommand(m_ArmControls);
    m_IntakeShooter.setDefaultCommand(m_IntakeShooterControls);
    // Configure the button bindings
    configureBindings();
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
