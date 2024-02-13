// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;
// import frc.robot.subsystems.Limelight;

import java.time.Year;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final XboxController m_controller;
  private final Drivetrain m_drive;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithGamepad(Drivetrain drive, XboxController controller) {
    m_drive = drive;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveWithJoystick(Drivetrain.isFieldOriented());
   // testLimelight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double yAxisValue = m_controller.getLeftX();
    double xAxisValue = m_controller.getLeftY();
    double twistAxisValue = m_controller.getRightX();
    SmartDashboard.putString("controller", String.format("X: %1.2f, Y: %1.2f, Z: %1.2f", xAxisValue, yAxisValue, twistAxisValue));
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward. 
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xAxisValue, 0.2)) * Constants.kMaxVelocity;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(yAxisValue, 0.2)) * Constants.kMaxVelocity;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(twistAxisValue, 0.2), 5)) * Constants.kMaxAngularVelocity;
    /*if (DriveToTarget.currentMode != DriveToTarget.targetFound)*/ {
      m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
    // m_drive.driveForwardAll(xSpeed/10);
    // m_drive.turnAroundAll(rot/50);
  }

  public void logJoystick() {
    
  }
  /*public void testLimelight() {
    if (m_controller.getAButtonPressed()) {
      if (DriveToTarget.getMode() == DriveToTarget.driverCam){
      DriveToTarget.setMode(DriveToTarget.looking);
      
      } else {
        
      }
    } else if (m_controller.getBButtonPressed()) {

    } else if (m_controller.getXButtonPressed()) {

    } else if (m_controller.getYButtonPressed()) {
        }
  }*/
}

