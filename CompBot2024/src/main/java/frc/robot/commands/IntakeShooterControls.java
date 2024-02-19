// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class IntakeShooterControls extends Command {

  private final XboxController m_controller;
  private final IntakeShooter m_intakeShooter;

  /** Creates a new IntakeControls. */
  public IntakeShooterControls(IntakeShooter intakeShooter, XboxController controller) {
    m_controller = controller;
    m_intakeShooter = intakeShooter;
    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightBumperPressed()) {
      m_intakeShooter.intake();
    } 
    if (m_controller.getRightBumperReleased()) {
      m_intakeShooter.stopIntake();
    }

    if (m_controller.getLeftBumperPressed()) {
      m_intakeShooter.shoot();
    }
    if (m_controller.getLeftBumperReleased()) {
      m_intakeShooter.stopShoot();
    }
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
