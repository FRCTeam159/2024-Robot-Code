// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivePath extends CommandBase {

  private Drivetrain m_drive;
  private Trajectory m_trajectory;
  private final Timer m_timer = new Timer();
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;

  double maxV;
  double maxA;
  double last_time;

  private String file;
  private PathPlannerPath pathFile;
  private List<PathPoint> pathPoints;
  private PathConstraints constraints;
  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  private final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
    new PIDConstants(4,0,0), new PIDConstants(4,0,0), Drivetrain.kMaxVelocity, 0.337);
    // 1:1.6,0.6,-10.0; 2:1.8,0.8,21.2; 2,2,1:1.8,0.7,-0.2; 3,3,1:1.9,0.8,13.8/3.0; 3.5,3.5,1:1.9,0.8,4.3; 4,4,1:1.9,0.9,-10.2

  public DrivePath(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.reset();
    System.out.println("DRIVEPATH_INIT");
    
    maxV=Drivetrain.kMaxVelocity;
    maxA=Drivetrain.kMaxAcceleration;

    m_trajectory=getTrajectory();
    if(m_trajectory ==null){
      System.out.println("failed to create Trajectory");
      return;
    }

    
    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();
    System.out.format("runtime:%-3.1f number states:%d intervals:%d\n",runtime,states,intervals); 
   // m_drive.resetOdometry();

    //System.out.println(p);

    m_timer.reset();
    m_timer.start();
    
    //m_drive.startAuto();
    elapsed=0;
  
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }


  Trajectory getTrajectory() {
      return pathPlannerTest();
  }

  Trajectory pathPlannerTest() {
    try {
      file = "ForwardPath"; //m_drive.centerPosition()?"Center":"NotCenter";
      pathFile = PathPlannerPath.fromPathFile(file);
      pathPoints = pathFile.getAllPathPoints();
      System.out.println("[][][][][][][][][][][][][][][][][]Number of points = " + pathPoints.get(0));
      constraints = new PathConstraints(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration, Drivetrain.kMaxAngularVelocity, Drivetrain.kMaxAngularAcceleration); // max vel & accel
      path = PathPlannerPath.fromPathPoints(pathPoints, constraints, new GoalEndState(0, new Rotation2d(0/*Math.PI/2*/)/*pathPoints.get(pathPoints.size()-1).rotationTarget.getTarget()*/)); // creates path based on the pathpoints, the constraints, and info on the final velocity (0) and rotatio
      trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(0, 0, 0), new Rotation2d(0)/*pathPoints.get(0).rotationTarget.getTarget()*/);

        System.out.println("selecting auto path:"+file);
    
      Pose2d p0 = trajectory.getInitialTargetHolonomicPose();

      // Pathplanner sets 0,0 as the lower left hand corner (FRC field coord system) 
      // for Gazebo, need to subtract intitial pose from each state so that 0,0 is 
      // in the center of the field 

      List<PathPlannerTrajectory.State> states = trajectory.getStates();
      for(int i=0;i<states.size();i++){
        PathPlannerTrajectory.State state = states.get(i);
        Pose2d p=state.getTargetHolonomicPose();
        Rotation2d h=state.targetHolonomicRotation;

        Pose2d pr=p.relativeTo(p0);
        //if(i==0)
       //  pr=new Pose2d(pr.getTranslation(),new Rotation2d()); // 
        //state.holonomicRotation=h.plus(new Rotation2d(Math.toRadians(180))); // go backwards

        //Pose2d psi=state.poseMeters.relativeTo(p0);
        state.positionMeters=pr.getTranslation();
        state.heading=pr.getRotation();
        //System.out.println(state.poseMeters);
      }
      List<State> stateList = new ArrayList<Trajectory.State>();
      for(int i=0;i<states.size();i++){
        PathPlannerTrajectory.State state = states.get(i);
        stateList.add(new State(state.timeSeconds, state.velocityMps, state.accelerationMpsSq, state.getTargetHolonomicPose(), state.curvatureRadPerMeter));
      }
      return new Trajectory(stateList);
    } catch (Exception ex) {
      System.out.println("failed to create pathweaver trajectory");
      ex.printStackTrace();
      return null;
    }
  }
  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    System.out.println("Alive");
    elapsed = m_timer.get();
    if(m_trajectory==null){
      System.out.print("ERROR DrivePath.execute - trajectory is null");
       return;
    }
    //elapsed = m_drive.getTime();

    PathPlannerTrajectory.State reference = trajectory.sample(elapsed);

    //System.out.format("Time:%-1.3f X Current X:%-1.2f Target:%-1.2f\n",
    //elapsed,m_drive.getPose().getX(),reference.poseMeters.getX()
   // );



    ChassisSpeeds speeds = m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), reference);
      m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

  }

    // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVEPATH_END");
    if (m_trajectory == null)
      return;
    //m_drive.endAuto();
    //TagDetector.setBestTarget();

    //m_drive.reset();
    // m_drive.enable();
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_trajectory == null){
      System.out.println("Failed to Create Trajectory");
      return true;
    }
    return (elapsed >= 1.3 * runtime || m_trajectory == null);

    //||m_drive.disabled()
  }
}
