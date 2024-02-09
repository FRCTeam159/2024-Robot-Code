// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AlignWheels extends Command {
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  double timeout;
  boolean done=false;
  /** Creates a new AlignWheels. */
  public AlignWheels(Drivetrain drive, double tm) {
    m_drive=drive;
    timeout=tm;
    m_timer.start();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetWheels(true);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_drive.wheelsReset())
      m_drive.resetWheels(false);
    else{
      //timeout=0.2;
      done=true;
      m_drive.resetOdometry();
      m_drive.resetPose(new Pose2d());
    }     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     if(!done)
      m_drive.resetOdometry();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(done || m_timer.get()>timeout)
      return true;
    return false;
  }
}