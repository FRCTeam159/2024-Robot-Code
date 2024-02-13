// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TargetMgr;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends Command {

  final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
      new PIDConstants(3, 0.0, 0), new PIDConstants(4, 0.0, 0.0), Drivetrain.kMaxVelocity, Drivetrain.kTrackRadius);

  Timer m_timer = new Timer();
  Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  static boolean debug = false;

  PathPlannerTrajectory m_pptrajectory;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  double yPath = 1;
  double xPath = 0;
  double rPath = 0;
  boolean m_reversed = false;
  boolean m_autoset = false;

  public DrivePath(Drivetrain drive, boolean rev) {
    m_reversed = rev;
    m_drive = drive;
    addRequirements(drive);
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    System.out.println("DRIVEPATH_INIT");
    m_autoset = Autonomous.getAutoset();
    m_ppcontroller.setEnabled(true);

    if (m_autoset) { // use apriltags or smartdashboard buttons
      Pose2d target = TargetMgr.getTarget();
      xPath = target.getX();
      yPath = target.getY();
      rPath = target.getRotation().getRadians();

      SmartDashboard.putNumber("xPath", xPath);
      SmartDashboard.putNumber("yPath", yPath);
      SmartDashboard.putNumber("rPath", rPath);
    } else { // set path manually
      xPath = SmartDashboard.getNumber("xPath", xPath);
      yPath = SmartDashboard.getNumber("yPath", yPath);
      rPath = SmartDashboard.getNumber("rPath", rPath);
    }
    m_timer.start();
    m_timer.reset();
    elapsed = 0;

    m_pptrajectory = pathplannerProgramPath();
    if (m_pptrajectory == null) {
      System.out.println("DrivePath - path creation failed !");
      return;
    }
    runtime = m_pptrajectory.getTotalTimeSeconds();
    states = m_pptrajectory.getStates().size();
    intervals = (int) (runtime / 0.02);

    // important ! otherwise get rotation glitch at start or reverse path
    m_ppcontroller.reset(m_drive.getPose(), new ChassisSpeeds());

    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    elapsed = m_timer.get();
    PathPlannerTrajectory.State pstate = m_pptrajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), pstate);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVEPATH_END");
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    return elapsed >= 1.3 * runtime || m_pptrajectory == null;
  }

  // *********************** trajectory functions *******************/
  // ===============================================
  // programPathPP: build a PathBuilder trajectory from variables
  // =================================================
  PathPlannerTrajectory pathplannerProgramPath() {
    List<Pose2d> points = new ArrayList<Pose2d>();
    double rpg = rPath;
    double rps = 0;

    if (m_reversed) { // go back to 0,0 !
      Pose2d pose = m_drive.getPose(); // start at current robot pose
      rpg = 0;
      rps = m_drive.getHeading();
      points.add(pose);
      points.add(new Pose2d());
    } else {
      points.add(new Pose2d()); // start at 0,0
      points.add(new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath)));
    }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(points);
    PathConstraints constraints = new PathConstraints(
        Drivetrain.kMaxVelocity,
        Drivetrain.kMaxAcceleration,
        Drivetrain.kMaxAngularVelocity,
        Drivetrain.kMaxAngularAcceleration);

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints, constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(rpg)));

    PathPlannerTrajectory traj = new PathPlannerTrajectory(path, new ChassisSpeeds(), Rotation2d.fromDegrees(rps));
    return traj;
  }

  // =================================================
  // reverse: reverse a set of points (drive backwards)
  // =================================================
  List<Pose2d> reverse(List<Pose2d> points) {
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
    List<Pose2d> newWaypoints = new ArrayList<>();
    for (Pose2d originalWaypoint : points)
      newWaypoints.add(originalWaypoint.plus(flip));
    return newWaypoints;
  }
}
