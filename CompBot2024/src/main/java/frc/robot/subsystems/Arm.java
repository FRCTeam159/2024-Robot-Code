// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BNO055;
import frc.robot.sensors.BNO055.BNO055OffsetData;

public class Arm extends SubsystemBase {
  public final double kMaxArmVelocity = 1200; // degrees/s
  public final double kMaxArmAcceleration = 1200; // degrees/s^2

  private final ArmFeedforward m_shoulderFeedforward = new ArmFeedforward(0.01, 0.05, 0.01);
  private final TrapezoidProfile.Constraints m_trapezoidConstraints = new TrapezoidProfile.Constraints(kMaxArmVelocity, kMaxArmAcceleration);
  private final ProfiledPIDController m_shoulderPIDController = new ProfiledPIDController(
    0.05, 0, 0,
    m_trapezoidConstraints
  );
  private final CANSparkMax m_shoulderMotor1;
  private final CANSparkMax m_shoulderMotor2;

  private double shoulderAngleSetpoint = 0; // degrees
  public final double armMinAngle = 5; // degrees
  public final double armMaxAngle = 110; // degrees
  public boolean initialized = false;
  public boolean enabled = false;
  public double currentAngle = 0;  // degrees, updated every periodic

  private static final BNO055OffsetData bno2Offsets = new BNO055OffsetData(-19, 52, -13, -24, 0, -2, 2, -8, -53, -66, 591);
  public static BNO055 m_armGyro = new BNO055(
    I2C.Port.kMXP,
    BNO055.BNO055_ADDRESS_B,
    "BNO055-2",
    BNO055.opmode_t.OPERATION_MODE_NDOF,
    BNO055.vector_type_t.VECTOR_GRAVITY,
    bno2Offsets
  );

  private String name = "arm";

  /** Creates a new Arm. */
  public Arm() {
    m_shoulderMotor1 = new CANSparkMax(kShoulderMotor1, CANSparkLowLevel.MotorType.kBrushless);
    m_shoulderMotor2 = new CANSparkMax(kShoulderMotor2, CANSparkLowLevel.MotorType.kBrushless);
    m_shoulderMotor2.setInverted(true);  // technically not necessary since we invert in the follow command, but I'm leaving it in just in case
    m_shoulderMotor2.follow(m_shoulderMotor1, true);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    if (!initialized) {
      waitForGyroInit(); // Make sure the gyro is ready before we move
      return;
    }
    currentAngle = getAngleFromGyro();
    if (!enabled) {
      // set the setpoint to the current angle to prevent control windup
      setTargetAngle(currentAngle);
    }
    clampSetpoint(); // ensure the setpoint does not go beyond the allowed boundaries
    log();
    runShoulderMotor();
  }

  public void enable() {
    setTargetAngle(getAngleFromGyro());
    m_shoulderPIDController.reset(new TrapezoidProfile.State(shoulderAngleSetpoint, 0));
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }

  // Use this to move the setpoint by the given amount
  public void adjustAngle(double adjustment) {
    setTargetAngle(shoulderAngleSetpoint + adjustment);
  }

  // Use this to set the setpoint to the given angle
  public void setTargetAngle(double a){
    shoulderAngleSetpoint = a;
  }

  public void clampSetpoint() {
    if (shoulderAngleSetpoint < armMinAngle) {
      setTargetAngle(armMinAngle);
    }
    if (shoulderAngleSetpoint > armMaxAngle) {
      setTargetAngle(armMaxAngle);
    }
  }

  public double getTargetAngle(){
    return shoulderAngleSetpoint;
  }
  public double getAngleFromGyro() {
    // gyro is in gravity mode
    // get X and Z gravity vectors
    // Convert to angle
    // when Z == g, angle is 0 degrees (toward front of robot)
    double xGravity = m_armGyro.getVector()[0];
    double zGravity = m_armGyro.getVector()[2];
    //SmartDashboard.putNumber(name + "Shoulder x", xGravity);
    //SmartDashboard.putNumber(name + "Shoulder Z", zGravity);
    Rotation2d result = Rotation2d.fromRadians(Math.atan2(zGravity, xGravity));
    result = result.minus(Rotation2d.fromDegrees(90));
    return result.getDegrees(); 
  }

  private void waitForGyroInit() {
    if (!initialized && m_armGyro.isInitialized() && m_armGyro.isCalibrated()) {
      // Set the setpoint to the current position when initializing
      setTargetAngle(getAngleFromGyro());
      initialized = true;
    }
  }

  void log() {
    //m_armGyro.log();
    SmartDashboard.putNumber(name + " Shoulder angle", currentAngle);
    SmartDashboard.putNumber(name + " Shoulder setpoint", shoulderAngleSetpoint);
  }

  private void runShoulderMotor() {
    double pidCommand = m_shoulderPIDController.calculate(currentAngle, shoulderAngleSetpoint);
    double ffCommand = m_shoulderFeedforward.calculate(
      Rotation2d.fromDegrees(shoulderAngleSetpoint).getRadians(),
      Rotation2d.fromDegrees(m_shoulderPIDController.getSetpoint().velocity).getRadians()
    );
    double totalCommand = ffCommand + pidCommand;
    SmartDashboard.putNumber(name + " Shoulder command", totalCommand);
    m_shoulderMotor1.set(totalCommand);
  }
}
