// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.IntakeShooter;

public class Shoot extends Command{
  IntakeShooter m_shooter;
  boolean shooter_ready=false;
  boolean shooting=false;
  Timer m_timer=new Timer();
  boolean m_note_at_start=false;
  public Shoot(IntakeShooter shooter) {
    m_shooter=shooter;
    addRequirements(shooter);
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("Shoot.init");
    shooter_ready=false;
    shooting=false;
    m_note_at_start=m_shooter.noteAtIntake();
    m_timer.reset();
    m_shooter.setShooterOn(); // turn shootere on flywheels
    m_shooter.setIntakeOff(); // turn off intake rollers
    m_shooter.setPushOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!shooter_ready && m_shooter.atTargetSpeed()){
      shooter_ready=true;
      m_shooter.setPushOn(); // 
      m_timer.reset();
      shooting=true;
      Robot.status="Shooting";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.log("Shoot.end");
    m_shooter.setShooterOff();
    m_shooter.setPushOff();
   // m_shooter.setIntakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Autonomous.okToRun())
      return true;
    // turn on pusher wheels and wait a little to make sure note has cleared the shooter
    if(shooter_ready && m_timer.get()>2 && !m_shooter.noteAtShooter()) {
      Autonomous.log("Shoot - shot delivered");
      return true;
    }
    if(shooting && m_timer.get()>5){ // taking too long - something isn't right
      Autonomous.log("Shoot - timed out");
      return true;
  }
    if(!m_note_at_start){ // never had a note to start with
      Autonomous.log("Shoot - no note at start - aborting");
      return true;
    }
    return false;
  }
}
