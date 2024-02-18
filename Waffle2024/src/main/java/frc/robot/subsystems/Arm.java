// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.BNO055;
import frc.robot.sensors.BNO055.BNO055OffsetData;

public class Arm extends SubsystemBase {

  private static final BNO055OffsetData bno2Offsets = new BNO055OffsetData(-19, 52, -13, -24, 0, -2, 2, -8, -53, -66, 591);
  public static BNO055 m_armGyro = new BNO055(
    I2C.Port.kMXP,
    BNO055.BNO055_ADDRESS_A,
    "BNO055-2",
    BNO055.opmode_t.OPERATION_MODE_NDOF,
    BNO055.vector_type_t.VECTOR_EULER,
    bno2Offsets
  );
  
  private static boolean have_arm=false; // test first !
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
    setAngle(armAngle);
    log();
  }

  public void setAngle(double angle) {
    boolean newangle=angle !=armAngle;
    armAngle = angle;
    m_PID.setSetpoint(armAngle);
    double output = m_PID.calculate(getAngle());
    if(have_arm)
       m_armPosMotor.set(output); 
    if(newangle)
        System.out.format("Arm.setAngle target:%-1.1f corr:%-1.1f",armAngle,output);
  }

  public double getAngle() {
    return m_armPosMotor.getEncoder().getPosition() / Constants.kArmGearRatio * 360.0;
  }

  public void adjustAngle(double adjustment) {
    armAngle += adjustment;
  }

  public void setTargetAngle(double a){
    armAngle=a;
  }

  public double getTargetAngle(){
    return armAngle;
  }
  public double getAngleFromGyro() {
    return m_armGyro.getVector()[1]; // Y angle 
  }
  void log(){
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm Setpoint", armAngle);
    SmartDashboard.putBoolean("Sensor", input.get());
  }
}
