// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BNO055;
import frc.robot.sensors.BNO055.BNO055OffsetData;

public class Arm extends SubsystemBase {

  private final ArmFeedforward m_shoulderFeedforward = new ArmFeedforward(0.1, 1.0, 0.1);
  private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.01, 1);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.01, 1);

  private final PIDController m_shoulderPIDController = new PIDController(0.02, 0, 0);
  private final PIDController m_shooterPIDController = new PIDController(0.01, 0, 0);

  private final CANSparkMax m_shoulderMotor;
  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_shooterMotor1;
  private final CANSparkMax m_shooterMotor2;

  private double shoulderAngleSetpoint = 0; // degrees
  public final double armMinAngle = 5; // degrees
  public final double armMaxAngle = 110; // degrees
  public boolean initialized = false;
  public boolean enabled = false;
  public final double kMaxArmVelocity = 1; // degrees/s
  public double currentAngle = 0;  // degrees, updated every periodic

  DigitalInput noteSensor1 = new DigitalInput(1);
  DigitalInput noteSensor2 = new DigitalInput(2);
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
    m_shoulderMotor = new CANSparkMax(kShoulderMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(kIntakeMotor, CANSparkLowLevel.MotorType.kBrushed);
    m_intakeMotor.setSmartCurrentLimit(38);
    m_intakeMotor.setInverted(true);
    m_shooterMotor1 = new CANSparkMax(kShooterMotor1, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotor1.setInverted(true);
    m_shooterMotor2 = new CANSparkMax(kShooterMotor2, CANSparkLowLevel.MotorType.kBrushless);
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
    runIntake();
  }

  public void enable() {
    m_shoulderPIDController.reset();
    setTargetAngle(getAngleFromGyro());
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
    double shoulderCommand = m_shoulderFeedforward.calculate(
      Rotation2d.fromDegrees(shoulderAngleSetpoint).getRadians(),
      Rotation2d.fromDegrees(kMaxArmVelocity).getRadians()
    );
    shoulderCommand = m_shoulderPIDController.calculate(currentAngle, shoulderAngleSetpoint);
    SmartDashboard.putNumber(name + " Shoulder command", shoulderCommand);
    m_shoulderMotor.set(shoulderCommand);
  }

  private void runIntake() {
    boolean sensor1State = noteSensor1.get();
    boolean sensor2State = noteSensor2.get();
    if (!sensor1State) {
      // intake should run forward (in) if there is no note
      //m_intakeMotor.set(1);
    } else if (sensor1State && sensor2State) {
      // intake should run backward (out) if note sensor 2 is triggered
      //m_intakeMotor.set(-0.1);
    } else if (sensor1State && !sensor2State) {
      // intake should stop if only note sensor 1 is triggered
      //m_intakeMotor.set(0);
    }
  }
}
