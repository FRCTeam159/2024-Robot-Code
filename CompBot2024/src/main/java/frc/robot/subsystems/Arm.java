// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.kIntakeMotor;
import static frc.robot.Constants.kShooterMotor;
import static frc.robot.Constants.kShoulderMotor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BNO055;
import frc.robot.sensors.BNO055.BNO055OffsetData;

public class Arm extends SubsystemBase {

  private final ArmFeedforward m_shoulderFeedforward = new ArmFeedforward(0.1, 0.1, 0.1);
  private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.1, 1);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.1, 1);

  private final PIDController m_shoulderPIDController = new PIDController(0.01, 0, 0);
  private final PIDController m_shooterPIDController = new PIDController(0.01, 0, 0);

  private final CANSparkMax m_shoulderMotor;
  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_shooterMotor;

  private double shoulderAngleSetpoint = 0;

  DigitalInput input = new DigitalInput(1);
  private static final BNO055OffsetData bno2Offsets = new BNO055OffsetData(-19, 52, -13, -24, 0, -2, 2, -8, -53, -66, 591);
  public static BNO055 m_armGyro = new BNO055(
    I2C.Port.kMXP,
    BNO055.BNO055_ADDRESS_B,
    "BNO055-2",
    BNO055.opmode_t.OPERATION_MODE_NDOF,
    BNO055.vector_type_t.VECTOR_EULER,
    bno2Offsets
  );

  private String name = "arm";
  
  /** Creates a new Arm. */
  public Arm() {
    m_shoulderMotor = new CANSparkMax(kShoulderMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(kIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotor = new CANSparkMax(kShooterMotor, CANSparkLowLevel.MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getAngleFromGyro();
    SmartDashboard.putNumber(name + " Shoulder angle", currentAngle);
    SmartDashboard.putNumber(name + " Shoulder setpoint", shoulderAngleSetpoint);
    log();
    double shoulderCommand = m_shoulderFeedforward.calculate(shoulderAngleSetpoint, 1);
    shoulderCommand += m_shoulderPIDController.calculate(currentAngle, shoulderAngleSetpoint);
    m_shoulderMotor.set(shoulderCommand);
  }

  public void adjustAngle(double adjustment) {
    shoulderAngleSetpoint += adjustment;
  }

  public double getAngleFromGyro() {
    return m_armGyro.getVector()[1]; // Y angle 
  }

  void log() {
    //m_armGyro.log();
  }
}
