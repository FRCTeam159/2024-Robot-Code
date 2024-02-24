// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends Command {

  private final XboxController m_controller;
  private final Drivetrain m_drive;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  boolean m_aligning=false;
  AlignWheels m_align=null;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithGamepad(Drivetrain drive, XboxController controller) {
    m_drive = drive;
    m_controller = controller;
    m_align = new AlignWheels(m_drive, 2.0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double now=0;//WPIUtilJNI.now() * 1e-6;
    m_xspeedLimiter.reset(now);
    m_yspeedLimiter.reset(now);
    m_rotLimiter.reset(now);
    System.out.println("DriveWithGampad started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveWithJoystick(Drivetrain.isFieldOriented());
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
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2))
        * Drivetrain.kMaxVelocity;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2))
        * Drivetrain.kMaxVelocity;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

    if (m_controller.getRightStickButtonPressed()) {
      System.out.println("Aligning");
      if (!m_aligning) {
        m_align.initialize();
        m_aligning = true;
      } else
        m_aligning = false;
    }
    if (m_aligning)
      align();
    if (!m_aligning) {
      final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.2))*Drivetrain.kMaxAngularAcceleration;
      m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    } 
  }

  void align() {
    if (m_align.isFinished()) {
      m_align.end(false);
      m_drive.resetOdometry();
      m_aligning = false;
    } else
      m_align.execute();
  }
}
