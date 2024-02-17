// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Wait extends Command {
  Drivetrain m_drive;
  double m_timeout;
  Timer m_timer = new Timer();
  /** Creates a new Wait. */
  public Wait(Drivetrain drive, double tm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_timeout = tm;
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Debug
    System.out.println("-");
    System.out.println("Waiting started");
    System.out.println("-");

    // Reset timer
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0.01, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Debug
    System.out.println("-");
    System.out.println("Waiting done");
    System.out.println("-");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_timeout;
  }
}
