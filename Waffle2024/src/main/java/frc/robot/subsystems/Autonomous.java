// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveProgram;
//import frc.robot.commands.ShootNote;

public class Autonomous extends SubsystemBase {
  public static boolean autoReset = false;
  Drivetrain m_drive;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PATHPLANNER = 2;
  public static final int AUTOTEST = 3;

  static public int trajectory_option = PATHPLANNER;

  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();

  static double XF=0.95;
  static double YF=-1.6;
  static double RF=-60;

  static double YR=-0.5;
  static double XR=-1.65;
  static double RR=60;

  static double xp=2;
  static double yp=0;
  static double rp=0;

  // Commands
  //public final ShootNote m_shootNote = new ShootNote();

  /** Creates a new Autonomous. */
  public Autonomous(Drivetrain drive) {
    m_drive = drive;
	  m_path_chooser.setDefaultOption("Pathplanner", PATHPLANNER);
    m_path_chooser.addOption("Program", PROGRAM);
    m_path_chooser.addOption("AutoTest", AUTOTEST);
   // m_path_chooser.addOption("Calibrate", CALIBRATE);

    SmartDashboard.putData(m_path_chooser);

    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

    m_alliance_chooser.setDefaultOption("Blue", TargetMgr.BLUE);
    m_alliance_chooser.addOption("Red", TargetMgr.RED);
    SmartDashboard.putData(m_alliance_chooser);

    m_position_chooser.setDefaultOption("Outside", TargetMgr.OUTSIDE);
    m_position_chooser.addOption("Center", TargetMgr.CENTER);
    m_position_chooser.addOption("Inside", TargetMgr.INSIDE);
    
    SmartDashboard.putData(m_position_chooser);

  }
  static public int getAlliance(){
    return m_alliance_chooser.getSelected();
  }
  static public int getPosition(){
    return m_position_chooser.getSelected();
  }
  public SequentialCommandGroup getCommand(){
    switch(trajectory_option){
      default:
      case PATHPLANNER:
        return new SequentialCommandGroup(drivePathplanner());
      case PROGRAM:
        return new SequentialCommandGroup(driveProgram());
      case AUTOTEST:
         //TODO create command
         return null;
      case CALIBRATE:
         //TODO create command
         return null;
    }
  }
  @Override
  public void periodic() {
  }

  public Command driveProgram() {
    return new DriveProgram(m_drive,false,false);
  }
 public Command drivePathplanner() {
    // An example command will be run in autonomous
    m_drive.resetPose(new Pose2d());
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.

    // Commands
    //NamedCommands.registerCommand("shootNote", m_shootNote);

    return AutoBuilder.buildAuto("BlueCenter");
  }
}