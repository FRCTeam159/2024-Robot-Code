// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class Shoot extends Command {
  Drivetrain m_drive;
  Arm m_arm;
  double m_timeout=2; // shoot emulation
  Timer m_timer = new Timer();
  
  /** Creates a new Shoot. */
  public Shoot(Arm arm, Drivetrain drive) {
    addRequirements(drive);
    m_arm = arm;
    m_drive=drive;
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("Shoot.start");
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(-0.001, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.log("Shoot.end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_timeout;
  }
}

