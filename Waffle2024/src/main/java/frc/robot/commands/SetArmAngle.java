// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmAngle extends Command {
  Arm m_arm;
  double m_angle;

  /** Creates a new SetArmAngle. */
  public SetArmAngle(Arm arm, double angle) {
    m_arm=arm;
    m_angle=angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Debug
    System.out.println("SetArmAngle.start");

    // Set arm angle
    m_arm.setTargetAngle(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Debug
    System.out.println("SetArmAngle.end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atTargetAngle();
  }
}