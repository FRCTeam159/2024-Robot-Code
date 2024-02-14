// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

public class DriveGyro {
  static public enum gyros {
    FRC450,
    NAVX,
    BNO55  
  } ;
  gyros gyro_type=gyros.FRC450;
  String gyro_name;
  AHRS navx=null;
  ADXRS450_Gyro adx450=null;
  BNO055 bno=null;
  
  /** Creates a new Gyro. */
  public DriveGyro(gyros type) {
    gyro_type=type;
    switch(type){
      default:
      case FRC450:
        adx450= new ADXRS450_Gyro();
        gyro_name="FRC450";
        break;
      case NAVX:
        navx= new AHRS(SerialPort.Port.kUSB);
        gyro_name="NAVX";
        break;
      case BNO55:
        {
          int[] bnoOffsets = {0, -42, -8, -24, -3, 0, 2, 299, -59, -25, 523};
          bno=BNO055.getInstance(
            BNO055.opmode_t.OPERATION_MODE_NDOF,
            BNO055.vector_type_t.VECTOR_EULER,
            I2C.Port.kMXP,
            BNO055.BNO055_ADDRESS_A,
            bnoOffsets
            );
        }
        gyro_name="BN055";
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
    switch(gyro_type){
      default:
      case FRC450:
        adx450.reset();break;
      case NAVX:
        navx.reset();break;
      case BNO55:
        bno.reset();break;
    }
  }

  public double getAngle() {
    switch(gyro_type){
      default:
      case FRC450:
        return -adx450.getAngle();
      case NAVX:
        return -navx.getAngle();
      case BNO55:
        return bno.getAngle();
    }
  }
  
}
