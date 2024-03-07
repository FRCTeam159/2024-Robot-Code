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

  boolean m_enable_align=false;

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
    double now = 0;// WPIUtilJNI.now() * 1e-6;
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
    double driveSpeed = 0.75; // Driving speed
    double rotSpeed = 0.2; // Old: 0.1 // Rotation speed
    double driveDeadband = 0.4;
    double rotDeadband = 0.2;

    SmartDashboard.putString("controller",
        String.format("X: %1.2f, Y: %1.2f, Z: %1.2f", xAxisValue, yAxisValue, twistAxisValue));
    var xSpeed = -driveSpeed*m_xspeedLimiter.calculate(Math.pow(MathUtil.applyDeadband(xAxisValue, driveDeadband), 3)) * Drivetrain.kMaxVelocity;
    var ySpeed = -driveSpeed*m_yspeedLimiter.calculate(Math.pow(MathUtil.applyDeadband(yAxisValue, driveDeadband), 3)) * Drivetrain.kMaxVelocity;
    var rot = -Drivetrain.kMaxAngularVelocity;

    boolean m_dervish_mode=false; // set this to try and spin off a stuck note

    // for testing auto routines we need to realign wheels at autonomous start or redeploy the code
    // - this is because the optimizer may have switched some of the wheels 180 
    // note: this mode may be disabled by setting m_enable_align to false above (default case for compbot)
    
    if (m_controller.getRightStickButtonPressed()) {
      if (m_enable_align) {
        System.out.println("Aligning");
        if (!m_aligning) {
          m_align.initialize();
          m_aligning = true;
        } else {
          m_aligning = false;
          m_align.end(true);
        }
      }
      else{
        System.out.println("Spinning");
        m_dervish_mode=true;
      }
    }
    if(m_dervish_mode && m_controller.getRightStickButtonReleased())
      m_dervish_mode=false;
    
    if (m_aligning)
      align();
    boolean do_targeting = TagDetector.isTargeting();
    if (!m_aligning && !do_targeting) {
      if(m_dervish_mode)
        rot*= twistAxisValue;
      else
        rot*= rotSpeed*m_rotLimiter.calculate(Math.pow(MathUtil.applyDeadband(twistAxisValue, rotDeadband), 3));
      m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
  }

  void align() {
    if (m_align.isFinished()) {
      m_align.end(false);
      m_aligning = false;
    } else
      m_align.execute();
  }
}
