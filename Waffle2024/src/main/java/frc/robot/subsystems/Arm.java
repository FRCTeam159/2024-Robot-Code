// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private static boolean have_arm=false;
  private CANSparkMax m_armPosMotor=null;
  private PIDController m_PID=null;
  
  private double armAngle = 90;
  static boolean sensor_test = true;
  boolean sensor_state = false;

  DigitalInput input = new DigitalInput(1);

  /** Creates a new Arm. */
  public Arm() {
    m_armPosMotor=new CANSparkMax(Constants.kSpareSpark,CANSparkLowLevel.MotorType.kBrushless);
    m_PID = new PIDController(0.05, 0, 0);
    m_PID.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", getAngle());
    SmartDashboard.putBoolean("Sensor", input.get());
    setAngle(armAngle);
  }

  public void setAngle(double angle) {
    this.armAngle = angle;
    m_PID.setSetpoint(angle);
    double output = m_PID.calculate(getAngle());
     m_armPosMotor.set(output);

    // System.out.println("Angle is being set : " + angle + " " + output + " " +
    // m_armPosMotor.getEncoder().getPosition());
  }

  public double getAngle() {
    return m_armPosMotor.getEncoder().getPosition() / Constants.kArmGearRatio * 360.0;
  }
}
