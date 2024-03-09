
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmControls;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.TestCommands;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.TargetMgr;
import frc.robot.subsystems.TestMotor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  //Subsystems

  public static final boolean have_arm=false; // test first !
  private final Drivetrain m_drivetrain = new Drivetrain();
  
  private static final boolean have_test = true;
  private TestMotor m_test = null;
  private Arm m_arm=null;
  private final Autonomous m_auto = new Autonomous(m_drivetrain, m_arm);
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller);
  private final TagDetector m_detector= new TagDetector(m_drivetrain);

  public static boolean m_haveArm = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
    if(have_arm){
      m_arm = new Arm();
      m_arm.setDefaultCommand(new ArmControls(m_arm, m_drivetrain,m_controller));
    }
    else if(have_test){
      m_test = new TestMotor();
      m_arm.setDefaultCommand(new TestCommands(m_test, m_controller));
    }
    // Add commands to PathPlanner
    NamedCommands.registerCommand("Wait", new Wait(m_drivetrain, 2.0));
  }

  public Command getAutonomousCommand() {
      return m_auto.getCommand();
  }
 
  public void robotInit() {
      TargetMgr.init();
      m_drivetrain.resetOdometry();
      m_detector.start();
  }

  public void teleopInit() {
    Robot.status = "Teleop";
    Autonomous.end(); 
  }

  public void autonomousInit() {
    TargetMgr.reset();
    Robot.status = "Autonomous";
    Autonomous.start();
    //m_Drivetrain.resetOdometry();
  }

  public void disabledInit() {
    Robot.status = "Disabled";
     Autonomous.end(); 
  }
 
}