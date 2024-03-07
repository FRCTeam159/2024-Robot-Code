// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeShooter;

public class IntakeShooterControls extends Command {

  private final XboxController m_controller;
  private final IntakeShooter m_intakeShooter;
  private final Arm m_arm;

  boolean m_shooting=false;
  boolean m_pickup=false;
  boolean m_eject=false;
  

  Shoot shoot;
  Pickup pickup;

  boolean testmode=false;

  /** Creates a new IntakeControls. */
  public IntakeShooterControls(IntakeShooter intakeShooter, Arm arm, XboxController controller) {
    m_controller = controller;
    m_intakeShooter = intakeShooter;
    m_arm=arm;
    shoot=new Shoot(intakeShooter);
    pickup=new Pickup(intakeShooter,arm);

    addRequirements(intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.enable();
    m_intakeShooter.setIntakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightBumperPressed()) {
      if (!m_shooting) {
        shoot.initialize();
        m_shooting = true;
      } else {
        shoot.end(true);
        m_shooting = false;
      }
    } else if (m_controller.getLeftBumperPressed()) {
      if (!m_pickup) {
        pickup.initialize();
        m_pickup = true;
      } else {
        pickup.end(true);
        m_pickup = false;
      }
    } else if (m_controller.getLeftStickButtonPressed()) {
      if (!m_eject) {
        m_eject = true;
        m_intakeShooter.setShooterOn();
        m_intakeShooter.setPushOn();
      } else {
        m_eject = false;
        m_intakeShooter.setShooterOff();
        m_intakeShooter.setPushOff();
      }
    }
    if (m_shooting)
      shoot();
    else if (m_pickup)
      pickup();
  }

  void shoot() {
    if (shoot.isFinished()) {
      shoot.end(false);
      m_shooting = false;
    } else
      shoot.execute();
  }
  void pickup(){
     if(pickup.isFinished()) {
      pickup.end(false);
      m_pickup = false;
    } else
      pickup.execute();
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
