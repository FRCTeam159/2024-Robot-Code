// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_climberMotor;

  boolean m_enabled=false;  // test feedback first 
  boolean m_climbing=false;
  double m_climb_increment=0.01;

  double m_position=0;

  PIDController m_pid=new PIDController(0.1,0.01,0);

  static double m_max=16.0; // highest position (inches)
  static double m_min=0.0;  // lowest position (inches)
  RelativeEncoder m_encoder;
  double inchesPerRotation=6.28/400;

  SparkLimitSwitch m_upperLimit;
  SparkLimitSwitch m_lowerLimit;

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor = new CANSparkMax(Constants.kClimber, CANSparkLowLevel.MotorType.kBrushless);
    m_encoder=m_climberMotor.getEncoder();
    m_upperLimit=m_climberMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    m_lowerLimit=m_climberMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    m_pid.setIntegratorRange(0, 0.3);
  }

  public double position(){
    return inchesPerRotation*m_encoder.getPosition();
  }

  public void reset(){
    m_position=0;
  }

  @Override
  public void periodic() {
    m_pid.setSetpoint(m_position);
    double v=m_pid.calculate(position(),m_position);
    if(m_enabled)
      m_climberMotor.setVoltage(v);
    String s=String.format("P:%-3.1f T:%-3.1f C:%-3.2f",position(),m_position,v);
    SmartDashboard.putString("Climber",s);
    log();
  }

  public boolean atTop(){
    return m_upperLimit.isPressed();
  }
  public boolean atBottom(){
    return m_lowerLimit.isPressed();
  }

  public void enable(){
     m_climbing=true;
  }
  public void disable(){
     m_climbing=false;
  }
  public void climbUp(){
    if(!atTop()){
      m_position+=m_climb_increment;
      m_position=m_position>m_max?m_max:m_position;
      m_climbing=true;
    }
  }
  public void climbDown(){
    if(!atBottom()){
      m_position-=m_climb_increment;
      m_position=m_position<m_min?m_min:m_position;
      m_climbing=true;
    }
  }

  void log(){
    SmartDashboard.putBoolean("ClimberUpperLimit",atTop());
    SmartDashboard.putBoolean("ClimberLowerLimit",atBottom());
  }
}
