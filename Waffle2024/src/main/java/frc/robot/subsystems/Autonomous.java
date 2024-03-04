// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AlignWheels;
import frc.robot.commands.DrivePath;
import frc.robot.commands.InitAuto;
import frc.robot.commands.PickUp;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.Shoot;
import frc.robot.commands.Wait;

public class Autonomous extends SubsystemBase {
  public static boolean autoReset = false;
  Drivetrain m_drive;
  Arm m_arm;

  public static final int PROGRAM = 1;
  public static final int PATHPLANNER = 2;
  public static final int AUTOTEST = 3;

  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();

  static double xp=TargetMgr.XF;
  static double yp=TargetMgr.YF;
  static double rp=TargetMgr.RF;

  static boolean m_reverse=false;
  static boolean m_autoselect=true;
  static boolean m_usetags=false;
  static boolean m_plotpath=true;
  static boolean m_pathplanner=false;

  static boolean m_ok2run = false;
  static boolean m_inAuto = false;
  static double last_time = 0;

  static Timer m_timer=new Timer();

   /** Creates a new Autonomous. 
   * @param m_arm */
  public Autonomous(Drivetrain drive, Arm arm) {
    m_drive = drive;
    m_arm=arm;

    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

	  m_path_chooser.setDefaultOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("Program", PROGRAM);
    //m_path_chooser.addOption("Path", PATHPLANNER);
    SmartDashboard.putData(m_path_chooser);
  
    m_alliance_chooser.setDefaultOption("Blue", TargetMgr.BLUE);
    m_alliance_chooser.addOption("Red", TargetMgr.RED);
    SmartDashboard.putData(m_alliance_chooser);

    m_position_chooser.setDefaultOption("Outside", TargetMgr.OUTSIDE);
    m_position_chooser.addOption("Center", TargetMgr.CENTER);
    m_position_chooser.addOption("Inside", TargetMgr.INSIDE);
    SmartDashboard.putData(m_position_chooser);

    SmartDashboard.putBoolean("Reverse",m_reverse);
    SmartDashboard.putBoolean("Autoset",m_autoselect);
    SmartDashboard.putBoolean("UseTags",m_usetags);
    SmartDashboard.putBoolean("Plot",m_plotpath);
    SmartDashboard.putBoolean("Pathplanner",m_pathplanner);
    SmartDashboard.putBoolean("OkToRun", okToRun());

    m_timer.start();
  }

  public static double autoTime(){
		return m_timer.get();
	}
  public static void log(String msg){
    double tm=autoTime();
    System.out.format("%-2.3f (%-2.3f) %s\n",tm,tm-last_time,msg);
    last_time=tm;
    SmartDashboard.putBoolean("OkToRun", okToRun());
  }

  // These methods are used to terminate auto if any command fails
  public static void start(){
    last_time=0;
    m_ok2run=true;
    m_inAuto=true;
    m_timer.reset();
    log("Auto Start");
  }
   public static void end(){
    last_time=0;
    m_inAuto=false;
    m_ok2run=true;
    log("Auto End");
  }
  public static boolean okToRun(){
    if(!m_inAuto)
      return true;
    return m_ok2run;
  }
  public static void stop(){ // called by failing command
   log("Auto error - aborting !!");
   m_ok2run=false;
  }

  static public int getAlliance(){
    return m_alliance_chooser.getSelected();
  }
  static public int getPosition(){
    return m_position_chooser.getSelected();
  }
  static public boolean getReverse(){
    return SmartDashboard.getBoolean("Reverse",m_reverse);
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
  static public boolean getUsePathplanner(){
    return SmartDashboard.getBoolean("Pathplanner",m_pathplanner);
  }
  
  
  public SequentialCommandGroup getCommand(){
    return new SequentialCommandGroup(
      // new InitAuto(m_arm), 
      getAutoCommand());
  }
  private SequentialCommandGroup getAutoCommand(){
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
          new AlignWheels(m_drive,2),
          //new Shoot(m_drive, 2.0),
          new ParallelCommandGroup(
            new DrivePath(m_drive,false)
            // Move arm to pickUp pos
            // new setAngle(Constants.kPickup);
            //new PickUp(m_arm, 2.0)
          ),
         //new AlignWheels(m_drive, 2.0),
          new ParallelCommandGroup(
            //new SetArmAngle(m_arm, Constants.kSpeaker),
            new DrivePath(m_drive,true)
          ),
          new DrivePath(m_drive,false)
          //new Shoot(m_drive, 2.0)
        );
      //case PATHPLANNER: /* Uses a command group from PathPlanner */
      //  return new SequentialCommandGroup(drivePathplanner());
    }
  }
  
  public Command drivePathplanner() {
    // An example command will be run in autonomous
    m_drive.resetPose(new Pose2d());
    String pathname="RightSideZeroed";
      return AutoBuilder.buildAuto(pathname);  
  }
}