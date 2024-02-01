// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveProgram;

public class Autonomous extends SubsystemBase {
  //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example path", new PathConstraints(4,3));
  public static boolean autoReset = false;
  Drivetrain m_drive;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PATHPLANNER = 2;
  public static final int AUTOTEST = 3;

  static public int trajectory_option = PATHPLANNER;

  SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();

  static double XF=0.95;
  static double YF=-1.6;
  static double RF=-60;

  static double YR=-0.5;
  static double XR=-1.65;
  static double RR=60;

  static double xp=XF;
  static double yp=YF;
  static double rp=RF;

  /** Creates a new Autonomous. */
  public Autonomous(Drivetrain drive) {
    m_drive = drive;
	  m_path_chooser.setDefaultOption("Pathplanner", PATHPLANNER);
    m_path_chooser.addOption("Program", PROGRAM);
    m_path_chooser.addOption("AutoTest", AUTOTEST);
   // m_path_chooser.addOption("Calibrate", CALIBRATE);

    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

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
    return new DriveProgram(m_drive);
  }
 public Command drivePathplanner() {
    // An example command will be run in autonomous
    m_drive.resetPose(new Pose2d());
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("ForwardPath");   
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
