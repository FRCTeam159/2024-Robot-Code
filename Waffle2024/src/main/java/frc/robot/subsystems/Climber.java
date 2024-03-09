// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_climberMotor;
  boolean m_up=true;
  boolean m_climbing=false;
  boolean m_climber_enabled=true; // Enable this to start climber
  double m_climb_value=2;
  Timer m_timer = new Timer();
  double m_rampTime = 1.0;
  boolean m_debug=true;
  /** Creates a new Climber. */
  public Climber() {
    if (m_climber_enabled) {
      m_climberMotor = new CANSparkMax(Constants.kClimber, CANSparkLowLevel.MotorType.kBrushless);
    } else {
      m_climberMotor = null;
    }
    m_timer.start();
  }
  

  @Override
  public void periodic() {
    if (!m_climber_enabled) {
      return;
    }
    if (m_climbing) {
      double value = m_climb_value; 
      double time = m_timer.get();
      if (time <= m_rampTime) {
        value = value*time/m_rampTime;
      }

      if (m_up) {
        m_climberMotor.set(value);
        if(m_debug)
            System.out.println("Climbing up "+value);
      } else {
        value=-m_climb_value;
        m_climberMotor.set(value);
        if(m_debug)
            System.out.println("Climbing down "+value);
      }
    } else 
      m_climberMotor.set(0);

  }

  public void enable(){
    m_climbing=true;
    //m_timer.reset();
  }
  public void disable(){
    m_climbing=false;
    m_timer.reset();
  }
  public void climbUp(){
    enable();
    m_up=true;
  }
  public void climbDown(){
    enable();
    m_up=false;
  }
}
