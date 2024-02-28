// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class DriveWithGamepad extends Command {

  private final XboxController m_controller;
  private final Drivetrain m_drive;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  boolean m_aligning = false;
  AlignWheels m_align = null;

  boolean m_autotest=true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithGamepad(Drivetrain drive, XboxController controller) {
    m_drive = drive;
    m_controller = controller;
    m_align = new AlignWheels(m_drive, 2.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_drive.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveWithJoystick(Drivetrain.isFieldOriented());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double yAxisValue = m_controller.getLeftX();
    double xAxisValue = m_controller.getLeftY();
    double twistAxisValue = m_controller.getRightX();
    SmartDashboard.putString("controller",
        String.format("X: %1.2f, Y: %1.2f, Z: %1.2f", xAxisValue, yAxisValue, twistAxisValue));
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xAxisValue, 0.2)) * Drivetrain.kMaxVelocity;
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(yAxisValue, 0.2)) * Drivetrain.kMaxVelocity;

    // for testing auto routines we need to realign wheels at autonomous start or redeploy the code
    // - this is because the optimizer may have switched some of the wheels 180 
    // this mode may be disabled for competition by setting m_autotest to false
    if (m_autotest && m_controller.getRightStickButtonPressed()) { 
      System.out.println("Aligning");
      if (!m_aligning) {
        m_align.initialize();
        m_aligning = true;
      } else {
        m_aligning = false;
        m_align.end(true);
      }
    }
    if (m_aligning)
      align();
    if (!m_aligning) {
      final var rot = -m_rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(twistAxisValue, 0.2), 5))
          * Drivetrain.kMaxAngularVelocity;
      /* if (DriveToTarget.currentMode != DriveToTarget.targetFound) */ {
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
      }
      // m_drive.driveForwardAll(xSpeed/10);
      // m_drive.turnAroundAll(rot/50);
    }
  }

  void align() {
    if (m_align.isFinished()) {
      m_align.end(false);
      m_aligning = false;
    } else
      m_align.execute();
  }

  public void logJoystick() {

  }

}
