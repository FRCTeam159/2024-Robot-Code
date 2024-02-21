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

  boolean shooting=false;
  boolean grabbing=false;

  Shoot shoot;
  Pickup pickup;

  boolean testmode=true;

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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (testmode) { // manually control shooting
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
    else { // use Pickup (left bumper) and Shoot(right bumper)commands
      if (m_controller.getRightBumperPressed()) {
        if (!shooting) {
          shoot.initialize();
          shooting = true;
        } else
          shooting = false;
      } else if (m_controller.getLeftBumperPressed()) {
        if (!grabbing) {
          pickup.initialize();
          grabbing = true;
        } else
          grabbing = false;
      }
      if (shooting)
        shoot();
      else if (grabbing)
        pickup();
    }
  }

  void shoot() {
    if (shoot.isFinished()) {
      shoot.end(false);
      shooting = false;
    } else
      shoot.execute();
  }
  void pickup(){
     if(pickup.isFinished()) {
      pickup.end(false);
      grabbing = false;
    } else
      pickup.execute();
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
