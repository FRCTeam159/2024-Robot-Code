// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;

public class Pickup extends Command{
  IntakeShooter m_shooter;
  Arm m_arm;
  boolean note_captured=false;

  public Pickup(IntakeShooter shooter, Arm arm) {
    m_shooter=shooter;
    m_arm=arm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake.init");
    m_shooter.setIntakeOn();
    Robot.status="Intake";
    note_captured=false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_shooter.noteAtIntake())
      note_captured=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake.end");
    m_shooter.setIntakeOff();
    m_arm.setTargetAngle(Constants.kSpeaker); // lift note off the ground
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {   
    if(note_captured && m_shooter.noteAtShooter()){
      return true;
    }
    return false;
  }
}
