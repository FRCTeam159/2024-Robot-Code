// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestMotor extends SubsystemBase {

  double m_speed = 0;

  /** Creates a new TestMotor. */

  private CANSparkMax m_testMotor = new CANSparkMax(Constants.kSpareSpark,CANSparkLowLevel.MotorType.kBrushed);

  public TestMotor() {}

  @Override
  public void periodic() {
    m_testMotor.set(m_speed);
    // This method will be called once per scheduler run
  }
  
  public void changevalue(double v){
    m_speed += v;
  }
    
  public void resetvalue(){
    m_speed = 0;
  }
}
