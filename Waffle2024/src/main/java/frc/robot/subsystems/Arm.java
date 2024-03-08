// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.BNO055;
import frc.robot.sensors.BNO055.BNO055OffsetData;

public class Arm extends SubsystemBase {
  static boolean m_navx=true;
  static double m_navx_offset=197.0; // observed gyro value when arm is horizontal

  
  double last_heading = 0;

  static AHRS m_NAVXgyro=new AHRS(SerialPort.Port.kUSB);

  static final BNO055OffsetData bno2Offsets = new BNO055OffsetData(-19, 52, -13, -24, 0, -2, 2, -8, -53, -66, 591);
  static final BNO055 m_armGyro = new BNO055(
    I2C.Port.kMXP,
    BNO055.BNO055_ADDRESS_A,
    "BNO055-2",
    BNO055.opmode_t.OPERATION_MODE_NDOF,
    BNO055.vector_type_t.VECTOR_GRAVITY,
    bno2Offsets
  );

  private static boolean have_arm=true; // test first !
  private CANSparkMax m_armPosMotor=null;
  private PIDController m_PID=null;

  boolean newAngle = true;
  
  private double armSetAngle = 90;
  static boolean sensor_test = true;
  boolean sensor_state = false;

  static final double MAX_ANGLE=100;
  static final double MIN_ANGLE=0;

  //DigitalInput input = new DigitalInput(1);
  private boolean m_initialized;

  /** Creates a new Arm. */
  public Arm() {
   if(have_arm){
      m_armPosMotor=new CANSparkMax(Constants.kSpareSpark,CANSparkLowLevel.MotorType.kBrushed);
      m_PID = new PIDController(0.02, 0, 0);
      m_PID.setTolerance(3.0);
      m_PID.reset(); 
    }
  }

 void setAngle() {
    if(!have_arm)
      return;
    m_PID.setSetpoint(armSetAngle);
    double current = getAngle();
    double output = m_PID.calculate(current);
    
    m_armPosMotor.set(output);
    if (newAngle){
      String s=String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current, armSetAngle, output);
      SmartDashboard.putString("Arm", s);
    }
    //System.out.println(s);  
    newAngle=false;
  }
  void setNewTarget(double angle){
    angle=angle>MAX_ANGLE?MAX_ANGLE:angle;
    angle=angle<MIN_ANGLE?MIN_ANGLE:angle;
    newAngle = Math.abs(angle-armSetAngle)>1e-6;
    armSetAngle=angle;
  }
  public double getAngle() {
    //double angle= -getAngleFromGyro()+180;
    double angle= getAngleFromGyro();
    angle=angle>MAX_ANGLE?MAX_ANGLE:angle;
    angle=angle<MIN_ANGLE?MIN_ANGLE:angle;
    return angle;
  }

  public void adjustAngle(double adjustment) {
    setNewTarget(armSetAngle+adjustment);
  }

  public void setTargetAngle(double a){
    setNewTarget(a);
  }

  public double getTargetAngle(){
    return armSetAngle;
  }

  public boolean atTargetAngle() {
    if(have_arm)
      return m_PID.atSetpoint();
    return true;
  }

  private void waitForGyroInit() {
    if (!m_initialized && m_armGyro.isInitialized() && m_armGyro.isCalibrated()) {
      // Set the setpoint to the current position when initializing
      //setTargetAngle(getAngleFromGyro());
      System.out.println("Gyro initialized");
      m_initialized=true;
    }
  }
 public double getAngleFromGyro() {
    
    if(m_navx){
      //return m_navx_offset-m_NAVXgyro.getRoll(); // returned values are negative
       double angle=m_NAVXgyro.getRoll()+m_navx_offset; // returned values are negative
       angle = unwrap(last_heading, angle);
       last_heading = angle;
       return angle;
    }
    else{ //BN0 
      // gyro is in gravity mode
      // get X and Z gravity vectors
      // Convert to angle
      // when Z == g, angle is 0 degrees (toward front of robot)
      double xGravity = m_armGyro.getVector()[0];
      double zGravity = m_armGyro.getVector()[2];
    
      Rotation2d result = Rotation2d.fromRadians(Math.atan2(zGravity, xGravity));
      result = result.minus(Rotation2d.fromDegrees(90));
      double a=result.getDegrees(); 

      a=a>180?180:a;
      a=a<0?180:a;
      return a;
    }
    
  }
  // removes heading discontinuity at 180 degrees
  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
  void log(){
    SmartDashboard.putNumber("Arm Angle", getAngleFromGyro());
    SmartDashboard.putNumber("Arm Setpoint", armSetAngle);
  }

  @Override
  public void periodic() {
    if (!m_navx && !m_initialized) 
      waitForGyroInit(); // Make sure the gyro is ready before we move
    else
      setAngle();
    log();
  }
}
