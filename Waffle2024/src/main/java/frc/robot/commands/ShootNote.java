// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootNote extends Command {
  // Variables
  Timer m_Timer = new Timer();
  
  /** Creates a new ShootNote. */
  public ShootNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    
    // Start timer
    m_Timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Debug
    System.out.println("Shooting");

    // Reset timer
    m_Timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 3.0;
  }
}