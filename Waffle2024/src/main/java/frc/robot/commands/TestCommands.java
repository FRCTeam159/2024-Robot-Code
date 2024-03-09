// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestMotor;

public class TestCommands extends Command {

  TestMotor m_test;
  XboxController m_controller;

  double m_MotorIncrement = 0.05;

  /** Creates a new TestCommands. 
   * @param m_controller 
   * @param m_test */
  public TestCommands(TestMotor test, XboxController controller) {
    m_test = test;
    m_controller = controller;
    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_test.resetvalue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_controller.getLeftTriggerAxis();
    double right = m_controller.getRightTriggerAxis();
    if(m_controller.getRightBumperPressed())
      m_test.resetvalue();
    else if (left > 0)
      m_test.changevalue(-left * m_MotorIncrement);
    else if (right > 0)
      m_test.changevalue(right * m_MotorIncrement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_test.resetvalue();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
