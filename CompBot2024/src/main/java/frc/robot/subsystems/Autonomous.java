// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DrivePath;
import frc.robot.commands.SetStartPose;
import frc.robot.commands.Pickup;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.Shoot;

public class Autonomous extends SubsystemBase {
  public static boolean autoReset = false;
  Drivetrain m_drive;
  Arm m_arm;
  IntakeShooter m_shooter;

  public static final int PROGRAM = 1;
  public static final int ONENOTEAUTO = 2;
  public static final int TWONOTEAUTO = 3;
  public static final int PATHPLANNER = 4;

  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();

  static double xp=TargetMgr.XF;
  static double yp=TargetMgr.YF;
  static double rp=TargetMgr.RF;

  static boolean m_autoselect=true;
  static boolean m_usetags=false;
  static boolean m_showtags = false;
  static boolean m_reverse=false;
  static boolean m_plotpath=true;
  static boolean m_pathplanner=true;
  static boolean m_ontarget=false;

  static boolean m_ok2run = false;
  static boolean m_inAuto = false;
  static double last_time = 0;

  static Timer m_timer=new Timer();

   /** Creates a new Autonomous. 
   * @param m_arm */
  public Autonomous(Drivetrain drive, Arm arm, IntakeShooter shooter) {
    m_drive = drive;
    m_arm=arm;
    m_shooter=shooter;
 
    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

	  m_path_chooser.setDefaultOption("TwoNoteAuto", TWONOTEAUTO);
    m_path_chooser.addOption("OneNoteAuto", ONENOTEAUTO);
    m_path_chooser.addOption("TestPath", PROGRAM);
   // m_path_chooser.addOption("Pathplanner", PATHPLANNER);
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
    SmartDashboard.putBoolean("ShowTags",m_showtags);
    SmartDashboard.putBoolean("Plot",m_plotpath);
    SmartDashboard.putBoolean("Pathplanner",m_pathplanner);
    SmartDashboard.putBoolean("OkToRun", okToRun());
    SmartDashboard.putBoolean("OnTarget", m_ontarget);

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
  public static void start(){ // set at start of autonomous
    last_time=0;
    m_ok2run=true;
    m_inAuto=true;
    m_timer.reset();
    log("Auto Start");
  }
   public static void end(){ // set at end of autonomous
    last_time=0;
    m_inAuto=false;
    m_ok2run=true;
    log("Auto End");
  }
  public static boolean okToRun(){ // each auto command should test this in isFinished
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
  static public boolean getShowtags(){
    return SmartDashboard.getBoolean("ShowTags",m_showtags);
  }
  static public void setOnTarget(boolean b){
    m_ontarget=b;
    SmartDashboard.putBoolean("OnTarget",b);
  }
  static public boolean getPlotpath(){
    return SmartDashboard.getBoolean("Plot",m_plotpath);
  }
  static public boolean getUsePathplanner(){
    return SmartDashboard.getBoolean("Pathplanner",m_pathplanner);
  }

  public SequentialCommandGroup getCommand(){
    return new SequentialCommandGroup(
      //new SetArmAngle(m_arm,Constants.kSpeaker),
      getAutoSequence());
  }
  private SequentialCommandGroup getAutoSequence(){
    int selected_path = m_path_chooser.getSelected();
    switch(selected_path){
      default:
      case PROGRAM: /* Uses values from SmartDashboard and can be reversed */
        return new SequentialCommandGroup(
          new DrivePath(m_drive, getReverse())
        );
      case TWONOTEAUTO: /* Uses alliance and position from SmartDashboard to create path */
         return twoNoteAuto();
      case ONENOTEAUTO: /* Uses alliance and position from SmartDashboard to create path */
         return oneNoteAuto();
      //case PATHPLANNER: /* Uses a command group from PathPlanner */
      //  return new SequentialCommandGroup(drivePathplanner());
    }
  }

  private SequentialCommandGroup oneNoteAuto() {
    return new SequentialCommandGroup(
        new SetStartPose(m_arm),
        new Shoot(m_shooter),
        new DrivePath(m_drive, false));
  }
  private SequentialCommandGroup twoNoteAuto() {
    return new SequentialCommandGroup(
        new SetStartPose(m_arm),
        new Shoot(m_shooter),
        new SetArmAngle(m_arm, Constants.kPickup),
        new ParallelCommandGroup(
            // At DrivePath.end set timer, if !hasNote, end auto
            new DrivePath(m_drive, false),
            new Pickup(m_shooter, m_arm)
        ),
        new ParallelCommandGroup(
            new DrivePath(m_drive, true),
            new SetArmAngle(m_arm, Constants.kSpeaker)
        ),
        new Shoot(m_shooter)
    );
  }
  
  public Command drivePathplanner() {
    // An example command will be run in autonomous
    m_drive.resetPose(new Pose2d());
    String pathname="RightSideZeroed";
    return AutoBuilder.buildAuto(pathname);
    
  }
}