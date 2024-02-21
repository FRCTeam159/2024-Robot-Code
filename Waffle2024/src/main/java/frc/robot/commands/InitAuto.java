// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class InitAuto extends Command {
  Arm m_arm;
  /** Creates a new InitAuto. 
   * @param m_arm */
  public InitAuto(Arm arm) {
    m_arm=arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Fix to be setAngle() command
    m_arm.setTargetAngle(Constants.kSpeaker);
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
    // If the arm is in the correct position, finish
    return m_arm.onTarget();
  }
}
