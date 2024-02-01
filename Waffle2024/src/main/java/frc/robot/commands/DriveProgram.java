// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveProgram extends Command {
  TrapezoidProfile.Constraints c=new TrapezoidProfile.Constraints(6.3, 3.15);
  private ProfiledPIDController ppc=new ProfiledPIDController(4, 0, 0,c);
  private HolonomicDriveController m_hcontroller=new HolonomicDriveController(new PIDController(0.5, 0, 0), new PIDController(0.5, 0, 0),ppc);

  private Drivetrain m_drive;

  Trajectory m_trajectory;

  int states;
  int intervals;

  double runtime;
  double elapsed = 0;

  double yPath = 4;
  double xPath = 4;
  double rPath = 90;

  boolean reversed = false;

  private Timer m_timer = new Timer();

  /** Creates a new DriveProgam. */
  public DriveProgram(Drivetrain drive) {
    m_drive=drive;
      addRequirements(drive);
  }
  public DriveProgram(Drivetrain drive,double x, double y, double r) {
    m_drive = drive;
    xPath=x;
    yPath=y; 
    rPath=r;    
    addRequirements(drive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DRIVEPATH_INIT");
    SmartDashboard.putNumber("xPath", xPath);
    SmartDashboard.putNumber("yPath", yPath);
    SmartDashboard.putNumber("rPath", rPath);

    if(xPath<0)
      reversed=true;
    else
      reversed=false;

    m_trajectory=programPath();
    if(m_trajectory ==null){
      System.out.println("failed to create Trajectory");
      return;
    } 
    
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsed = m_timer.get();

    if(m_trajectory==null){
      System.out.print("ERROR DrivePath.execute - trajectory is null");
       return;
    }

    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds= m_hcontroller.calculate(m_drive.getPose(), reference,reference.poseMeters.getRotation());
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 2*speeds.omegaRadiansPerSecond, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVEPATH_END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elapsed >= 1.3 * runtime || m_trajectory == null);
  }

  // =================================================
  // getTrajectory: return a selected trajectory
  // =================================================
  Trajectory programPath() {
    Pose2d pos1 = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d pos2 = new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath));

    List<Pose2d> points = new ArrayList<Pose2d>();

    points.add(pos1);
    points.add(pos2);
   
    return makeTrajectory(points, reversed);
  }
  // =================================================
  // makeTrajectory: build a trajectory from a list of poses
  // =================================================
  Trajectory makeTrajectory(List<Pose2d> points, boolean reversed) { 
    double maxV=Drivetrain.kMaxVelocity;
    double maxA=Drivetrain.kMaxAcceleration;

    TrajectoryConfig config = new TrajectoryConfig(maxV,maxA);
    config.setReversed(reversed);
   
    return TrajectoryGenerator.generateTrajectory(points, config);
  }


}
