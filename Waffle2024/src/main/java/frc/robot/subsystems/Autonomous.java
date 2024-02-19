// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Wait;

public class Autonomous extends SubsystemBase {
  public static boolean autoReset = false;
  Drivetrain m_drive;

  public static final int PROGRAM = 1;
  public static final int PATHPLANNER = 2;
  public static final int AUTOTEST = 3;

  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();

  static double xp=1.4;
  static double yp=-1.5;
  static double rp=-60;

  static boolean m_reversed=false;
  static boolean m_autoselect=true;
  static boolean m_usetags=false;
  static boolean m_plotpath=true;

   /** Creates a new Autonomous. 
   * @param m_arm */
  public Autonomous(Drivetrain drive, Arm arm) {
    m_drive = drive;
    Arm m_arm=arm;

    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

	  m_path_chooser.setDefaultOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("Program", PROGRAM);
    m_path_chooser.addOption("Pathplanner", PATHPLANNER);
    SmartDashboard.putData(m_path_chooser);
  
    m_alliance_chooser.setDefaultOption("Blue", TargetMgr.BLUE);
    m_alliance_chooser.addOption("Red", TargetMgr.RED);
    SmartDashboard.putData(m_alliance_chooser);

    m_position_chooser.setDefaultOption("Outside", TargetMgr.OUTSIDE);
    m_position_chooser.addOption("Center", TargetMgr.CENTER);
    m_position_chooser.addOption("Inside", TargetMgr.INSIDE);
    SmartDashboard.putData(m_position_chooser);

    SmartDashboard.putBoolean("Reversed",m_reversed);
    SmartDashboard.putBoolean("Autoset",m_autoselect);
    SmartDashboard.putBoolean("UseTags",m_usetags);
    SmartDashboard.putBoolean("Plot",m_plotpath);
  }
  static public int getAlliance(){
    return m_alliance_chooser.getSelected();
  }
  static public int getPosition(){
    return m_position_chooser.getSelected();
  }
  static public boolean getReverse(){
    return SmartDashboard.getBoolean("Reversed",m_reversed);
  }
  static public boolean getAutoset(){
    return SmartDashboard.getBoolean("Autoset",m_autoselect);
  }
  static public boolean getUsetags(){
    return SmartDashboard.getBoolean("UseTags",m_usetags);
  }
  static public boolean getPlotpath(){
    return SmartDashboard.getBoolean("Plot",m_plotpath);
  }
  public SequentialCommandGroup getCommand(){
    int selected_path = m_path_chooser.getSelected();
    switch(selected_path){
      default:
      case PROGRAM: /* Uses values from SmartDashboard and can be reversed */
        return new SequentialCommandGroup(
          //new AlignWheels(m_drive,2),
          new DrivePath(m_drive, getReverse())
        );
      case AUTOTEST: /* Uses alliance and position from SmartDashboard to create path */
         return new SequentialCommandGroup(
          //new AlignWheels(m_drive,2),
           new DrivePath(m_drive,false),
           new Wait(m_drive, 0.5),
           new DrivePath(m_drive,true)
         );
      case PATHPLANNER: /* Uses a command group from PathPlanner */
        return new SequentialCommandGroup(drivePathplanner());
    }
  }
  
  public Command drivePathplanner() {
    // An example command will be run in autonomous
    m_drive.resetPose(new Pose2d());
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.

    // Commands
    //NamedCommands.registerCommand("shootNote", m_shootNote);

    //return AutoBuilder.buildAuto("BlueCenter");

    // Paths
    // Load the path you want to follow using its name in the GUI

    String pathname="RightSideZeroed";
      return AutoBuilder.buildAuto(pathname);
    
  }
}