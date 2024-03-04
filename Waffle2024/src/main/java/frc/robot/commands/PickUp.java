// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

public class PickUp extends Command {
  Drivetrain m_drive;
  double m_timeout;
  Timer m_timer = new Timer();
  Arm m_arm;
  
  /** Creates a new PickUp. */
  public PickUp(Arm arm, double tm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_timeout = tm;
    m_timer.start();
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.log("Pickup.init");
    m_arm.setTargetAngle(Constants.kPickup);
    // Reset timer
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Resets wheels
    //m_drive.drive(0.0, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Autonomous.log("Pickup.end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Autonomous.okToRun())
      return true; 
    return m_arm.onTarget();
  }
}