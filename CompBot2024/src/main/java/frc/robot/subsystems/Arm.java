// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.kIntakeMotor;
import static frc.robot.Constants.kShooterMotor;
import static frc.robot.Constants.kShoulderMotor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BNO055;

public class Arm extends SubsystemBase {

  private final SimpleMotorFeedforward m_shoulderFeedforward = new SimpleMotorFeedforward(0.1, 1);
  private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.1, 1);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.1, 1);

  private final PIDController m_shoulderPIDController = new PIDController(0.01, 0, 0);
  private final PIDController m_shooterPIDController = new PIDController(0.01, 0, 0);

  private final CANSparkMax m_shoulderMotor;
  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_shooterMotor;

  private final BNO055 m_armGyro;

  DigitalInput input = new DigitalInput(1);
  
  /** Creates a new Arm. */
  public Arm() {
    int[] bnoOffsets = {0, -42, -8, -24, -3, 0, 2, 299, -59, -25, 523};
    m_armGyro=BNO055.getInstance(
      BNO055.opmode_t.OPERATION_MODE_NDOF,
      BNO055.vector_type_t.VECTOR_EULER,
      I2C.Port.kMXP,
      BNO055.BNO055_ADDRESS_B,
      bnoOffsets
    );  

    m_shoulderMotor = new CANSparkMax(kShoulderMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(kIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotor = new CANSparkMax(kShooterMotor, CANSparkLowLevel.MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
