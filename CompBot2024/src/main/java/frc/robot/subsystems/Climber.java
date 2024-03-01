// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_climberMotor;
  boolean m_up=true;
  boolean m_climbing=false;
  double m_climb_value=0.1;
  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor= new CANSparkMax(Constants.kClimber, CANSparkLowLevel.MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    if (m_climbing) {
      if (m_up)
        m_climberMotor.set(m_climb_value);
      else
        m_climberMotor.set(-m_climb_value);
    } else
      m_climberMotor.set(0);
  }

  public void enable(){
    m_climbing=true;
  }
  public void disable(){
    m_climbing=false;
  }
  public void climbUp(){
      enable();
      m_up=true;
  }
  public void climbDown(){
    disable();
    m_up=false;
  }
}
