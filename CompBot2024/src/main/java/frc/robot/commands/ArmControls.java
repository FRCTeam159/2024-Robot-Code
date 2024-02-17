// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmControls extends Command {

  private final XboxController m_controller;
  private final Arm m_arm;
  double m_arm_position;

  public static final double ARM_MOVE_RATE=0.5;

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
    double left=m_controller.getLeftTriggerAxis();
    double right=m_controller.getRightTriggerAxis();

    // if(m_controller.getRightBumperPressed())
    //   m_arm.togglePusher();
    // else if(m_controller.getLeftBumperPressed())
    //   m_arm.toggleShooter();
    if(m_controller.getAButtonPressed())
      m_arm.setTargetAngle(Constants.kPickup);   
    if(m_controller.getBButtonPressed())
      m_arm.setTargetAngle(Constants.kSpeaker);
    if(m_controller.getXButtonPressed())
      m_arm.setTargetAngle(Constants.kAmp);
    else if (left > 0)
      m_arm.adjustAngle(-left*ARM_MOVE_RATE);
    else if (right > 0)
      m_arm.adjustAngle(right*ARM_MOVE_RATE);
    
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
