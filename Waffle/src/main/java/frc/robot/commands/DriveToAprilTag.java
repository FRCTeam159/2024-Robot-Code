// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TargetMgr;
import frc.robot.subsystems.TargetMgr.TagTarget;

public class DriveToAprilTag extends CommandBase {
  public Limelight m_Limelight;
  public TargetMgr m_TargetMgr;
  public Drivetrain m_drive;
  
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(Limelight limelight, TargetMgr targetMgr, Drivetrain drive) {
    m_Limelight = limelight;
    m_TargetMgr = targetMgr;
    m_drive = drive;

    addRequirements(targetMgr);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
