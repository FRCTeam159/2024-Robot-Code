// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  public static boolean debug = false;
  String name;

  int cnt = 0;

  boolean m_inverted = false;
  boolean m_optimize = true;

  // PID controllers for drive and steer motors
  private final PIDController m_drivePIDController = new PIDController(4, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(
      0.5,
      0.01,
      0
  // new TrapezoidProfile.Constraints(
  // kMaxAngularSpeed, kMaxAngularAcceleration)
  );

  // private final PIDController m_turningPIDController = new PIDController(7, 0,
  // 0);

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoder
   */

  private int m_id;

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int id) {

    m_id = id;

    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

    name = Drivetrain.chnlnames[m_id - 1];

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Drivetrain.kDistPerRot); // inches to meters
    m_driveEncoder.setVelocityConversionFactor(Drivetrain.kDistPerRot / 60); // convert RPM to meters per second

    // Set the distance (in this case, angle) in radians per pulse for the turning
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(Drivetrain.kRadiansPerRot); // inches to meters
    m_turningEncoder.setVelocityConversionFactor(Drivetrain.kRadiansPerRot / 60); // convert RPM to meters per second
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(Math.toRadians(1.0));
  }

  public double getRotations() {
    return m_driveEncoder.getPosition() / m_driveEncoder.getPositionConversionFactor();
  }

  public void reset() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    System.out.println(name + " reset");
    cnt = 0;
  }

  void setOptimize(boolean t) {
    m_optimize = t;
  }

  public double heading() {
    return m_turningEncoder.getPosition();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(heading());
  }

  public void showWheelPosition() {
    System.out.format("%s %-3.1f\n", name, Math.toDegrees(heading()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDistance() {
    return m_driveEncoder.getPosition();
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation2d());
    // System.out.println("optimize = " + m_optimize);
    double velocity = getVelocity();
    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(velocity, state.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    double turn_angle = getRotation2d().getRadians();

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.calculate(turn_angle, state.angle.getRadians());
    double turnFeedforward = 0; //-m_turnFeedforward.calculate(state.angle.getRadians());

    double set_drive = driveOutput + driveFeedforward;
    double set_turn = turnOutput + turnFeedforward;

    driveForward(set_drive);
    m_turningMotor.set(set_turn);

    if (debug) {
      String s = String.format("Drive p:%-1.3f Angle t:%-3.3f a:%-2.3f c:%-2.3f\n",
          getDistance(), Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn);
      SmartDashboard.putString(name, s);
    }
  }

  public void log() {
    if (!debug) {
      String s = String.format("Drive:%-1.3f m Angle:%-4.1f Rotations:%-4.2f\n",
          getDistance(), getRotation2d().getDegrees(), getRotations());
      SmartDashboard.putString(name, s);
    }

  }

  public boolean isInverted() {
    return m_inverted;
  }

  public void setDriveInverted(boolean b) {
    m_inverted = b;
    m_driveMotor.setInverted(b);
  }

  public void driveForward(double dist) {
    // dist = m_inverted? -dist: dist;
    m_driveMotor.setVoltage(dist);
  }

  public void turnAround(double dist) {
    m_turningMotor.setVoltage(dist);
  }

  public void resetWheel() {
    setAngle(0, 0);
  }

  public boolean wheelReset() {
    return m_turningPIDController.atSetpoint();
  }

  // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d) {
    double r = Math.toRadians(a);
    m_turningPIDController.setSetpoint(r);
    double current = getRotation2d().getRadians(); // rotations in radians
    double turnOutput = m_turningPIDController.calculate(current, r);
    m_driveMotor.set(d);
    m_turningMotor.set(turnOutput);
  }

  @Override
  public void periodic() {
   // if (!debug)
   //   log();
  }

}
