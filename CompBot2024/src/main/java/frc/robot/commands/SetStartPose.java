// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TargetMgr;

public class SetStartPose extends Command {
  /** Creates a new InitArm. */

  Arm m_arm;
  Timer m_timer=new Timer();
  public SetStartPose(Arm arm) {
    m_arm=arm;
    m_timer.start();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.status="Auto Start";
    m_arm.enable();
    TargetMgr.clearStartPose();
    m_timer.reset();
    m_arm.setTargetAngle(Constants.kSpeaker);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get()>3) {
      // taking too long
      System.out.println("SetStartPose timeout expired");
      return true;
    }
    return m_arm.atTargetAngle()&&TargetMgr.startPoseSet();
  }
}
