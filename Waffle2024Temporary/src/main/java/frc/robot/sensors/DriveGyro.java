// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public class DriveGyro {
  static public enum gyros {
    FRC450,
    NAVX,
    BNO55  
  } ;
  gyros gyro_type=gyros.FRC450;
  String gyro_name;
  AHRS gyro;
  /** Creates a new Gyros. */
  public DriveGyro(gyros type) {
    gyro_type=type;
    switch(type){
      default:
      case NAVX:
        gyro= new AHRS(SerialPort.Port.kUSB);
        gyro_name="NAVX";
        break;
    }
  }
  public gyros getType(){
    return gyro_type;
  }
  public String toString(){
    return gyro_name;
  }
  public void reset() {
   gyro.reset();
  }
  public double getAngle() {
    switch(gyro_type){
      default:
      case FRC450:
        return -gyro.getAngle();
      case NAVX:
      return -gyro.getAngle();
      case BNO55:
        return gyro.getAngle();
    }
  }
  public double getRate() {
    return gyro.getRate();
  }
}
