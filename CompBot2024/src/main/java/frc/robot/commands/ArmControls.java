// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class ArmControls extends Command {
  private final XboxController m_controller;
  private final Arm m_arm;
  boolean m_targeting=false;
  AutoTarget target;

  public static final double ARM_MOVE_RATE=0.5;

  /** Creates a new ArmControls. 
   * @param m_Drivetrain */
  public ArmControls(Arm arm, Drivetrain drive, XboxController controller) {
    m_arm = arm;
    m_controller = controller;
    target=new AutoTarget(m_arm,drive);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.enable();
    m_targeting=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left=m_controller.getLeftTriggerAxis();
    double right=m_controller.getRightTriggerAxis();

    if(m_controller.getAButtonPressed()) {
      m_arm.setTargetAngle(Constants.kPickup); 
    } else if(m_controller.getYButtonPressed()) { 
      m_arm.setTargetAngle(Constants.kSpeaker);
    } else if(m_controller.getBButtonPressed())
      m_arm.setTargetAngle(Constants.kAmp);
    else if(m_controller.getXButtonPressed()){
        if(!m_targeting){
          target.initialize();
          m_targeting=true;
        }
        else
          m_targeting=true;
    } 
    else if (left > 0.1) {
      m_arm.adjustAngle(-left*ARM_MOVE_RATE);
    } else if (right > 0.1) {
      m_arm.adjustAngle(right*ARM_MOVE_RATE);
    }
    if(m_targeting)
      target();
  }

  void target() {
    if(target.isFinished()) {
     target.end(m_controller.getXButtonReleased());
     m_targeting = false;
   } else
     target.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
