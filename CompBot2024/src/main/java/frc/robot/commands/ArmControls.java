// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmControls extends Command {

  private final XboxController m_controller;
  private final Arm m_arm;

  /** Creates a new ArmControlls. */
  public ArmControls(Arm arm, XboxController controller) {
    m_arm = arm;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootTheRing();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void shootTheRing() {
    if (m_controller.getAButtonPressed()) {
       System.out.println("A button pressed");
    }
  }
}
