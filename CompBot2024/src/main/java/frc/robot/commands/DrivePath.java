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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TargetMgr;
import utils.PathData;
import utils.PlotUtils;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends Command {

  ArrayList<PathData> pathdata = new ArrayList<PathData>();

  final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
      new PIDConstants(2.0, 0.0, 0), new PIDConstants(4, 0.0, 0.0), Drivetrain.kMaxVelocity, Drivetrain.kTrackRadius);

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
  double scale=0.5;

  int plot_type = utils.PlotUtils.PLOT_NONE;
  //int plot_type = frc.robot.utils.PlotUtils.PLOT_LOCATION;

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
    if(Autonomous.getPlotpath())
      plot_type=utils.PlotUtils.PLOT_LOCATION;

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

    PlotUtils.initPlot();

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

    PlotUtils.setInitialPose(m_pptrajectory.sample(0).getTargetHolonomicPose(), Drivetrain.kFrontWheelBase);


    // important ! otherwise get rotation glitch at start or reverse path
    m_ppcontroller.reset(m_drive.getPose(), new ChassisSpeeds());

    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
if(m_pptrajectory==null)
      return;
    elapsed = m_timer.get();
    PathPlannerTrajectory.State pstate = m_pptrajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), pstate);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

    Trajectory.State reference = new Trajectory.State(pstate.timeSeconds, pstate.velocityMps, pstate.accelerationMpsSq,
        pstate.getTargetHolonomicPose(), pstate.curvatureRadPerMeter);
    
    if (plot_type == PlotUtils.PLOT_LOCATION)
      plotLocation(reference);
    else if (plot_type == PlotUtils.PLOT_DYNAMICS)
      plotDynamics(reference);
    else if (plot_type == PlotUtils.PLOT_POSITION)
      plotPosition(reference);
  }

  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVEPATH_END");

    if (plot_type != utils.PlotUtils.PLOT_NONE)
      utils.PlotUtils.publish(pathdata, 6, plot_type);
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
      if (m_drive.getDistance() < 0.2) // probably an error to start too close to 0 ?
        return null;
      Pose2d pose = m_drive.getPose(); // start at current robot pose
      rpg = 0;
      rps = m_drive.getHeading();
      points.add(pose);
      points.add(new Pose2d());
    } else {
      if (m_drive.getDistance() > 0.2) // probably an error to start too far from 0 ?
        return null;
      points.add(new Pose2d()); // start at 0,0
      points.add(new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath)));
    }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(points);
    PathConstraints constraints = new PathConstraints(
        scale * Drivetrain.kMaxVelocity,
        scale * Drivetrain.kMaxAcceleration,
        scale * Drivetrain.kMaxAngularVelocity,
        scale * Drivetrain.kMaxAngularAcceleration);

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints, constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(rpg)));

    PathPlannerTrajectory traj = new PathPlannerTrajectory(path, new ChassisSpeeds(), Rotation2d.fromDegrees(rps));
    return traj;
  }

  // *********************** plotting methods *******************/

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
// *********************** plotting methods *******************/

  // =================================================
  // plotPosition: collect PathData for motion error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] trackwidth
  // =================================================
  void plotPosition(Trajectory.State state) {
    // PathPlanner uses holonomicRotation for heading
    Rotation2d r;

    r = state.poseMeters.getRotation();
    Pose2d p = new Pose2d(state.poseMeters.getTranslation(), r);
    state.poseMeters = p;

    PathData pd = PlotUtils.plotPosition(
        state.timeSeconds,
        state.poseMeters,
        m_drive.getPose(),
        Drivetrain.kFrontWheelBase);
    pathdata.add(pd);
  }

  // =================================================
  // plotDynamics: collect PathData for dynamics error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] target velocity
  // arg[4] observed velocity
  // arg[5] target acceleration
  // =================================================
  void plotDynamics(Trajectory.State state) {
    PathData pd = PlotUtils.plotDynamics(
        state.timeSeconds,
        state.poseMeters, m_drive.getPose(),
        state.velocityMetersPerSecond, m_drive.getVelocity(),
        state.accelerationMetersPerSecondSq);
    pathdata.add(pd);
  }

  // =================================================
  // plotLocation: collect PathData for location error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target X
  // arg[2] observed X
  // arg[3] target Y
  // arg[4] observed Y
  // arg[5] target Heading
  // arg[5] observed Heading
  void plotLocation(Trajectory.State state) {
    PathData pd = new PathData();
    pd.tm = state.timeSeconds;
    Pose2d target_pose = state.poseMeters;
    Pose2d current_pose = m_drive.getPose();

    pd.d[0] = target_pose.getX();
    pd.d[1] = current_pose.getX();
    pd.d[2] = target_pose.getY();
    pd.d[3] = current_pose.getY();

    pd.d[4] = 2 * state.poseMeters.getRotation().getRadians();
    pd.d[5] = 2 * current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
