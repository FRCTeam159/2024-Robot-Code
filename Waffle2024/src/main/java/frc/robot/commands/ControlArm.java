// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ControlArm extends Command {
  double m_arm_position;
  Arm m_arm;
  XboxController m_controller;

  /** Creates a new ControlArm. */
  public ControlArm(Arm arm, XboxController controller) {
    m_arm=arm;
    m_controller=controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getAButtonPressed())
      m_arm_position=Constants.kPickup;   
    if(m_controller.getBButtonPressed())
      m_arm_position=Constants.kSpeaker;
    if(m_controller.getXButtonPressed())
      m_arm_position=Constants.kAmp;
     m_arm.setAngle(m_arm_position);
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
