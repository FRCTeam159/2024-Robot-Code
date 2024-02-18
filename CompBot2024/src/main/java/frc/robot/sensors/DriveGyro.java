// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.sensors.BNO055.BNO055OffsetData;
import static frc.robot.Constants.*;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveGyro {
  static public enum gyros {
    FRC450,
    NAVX,
    BNO55,
    PIGEON
  } ;
  gyros gyro_type=gyros.FRC450;
  String gyro_name;
  AHRS navx=null;
  ADXRS450_Gyro adx450=null;
  BNO055 bno=null;
  boolean bnoWasInitialized = false;
  Pigeon2 pigeon=null;

  
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
          BNO055OffsetData bno1Offsets = new BNO055OffsetData(-14, -27, -12, -24, -2, -1, 0, -139, -122, -475, 805);
          bno = new BNO055(
            I2C.Port.kMXP,
            BNO055.BNO055_ADDRESS_A,
            "BNO055-1",
            BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
            BNO055.vector_type_t.VECTOR_EULER,
            bno1Offsets
          );
        }
        gyro_name="BN055";
        break;
      case PIGEON:
        pigeon = new Pigeon2(kPigeonCanId);
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
      case PIGEON:
        pigeon.reset();
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
        if(!bnoWasInitialized && bno.isInitialized()) {
          bnoWasInitialized = true;
          bno.reset();
        }
        return -bno.getRotation2d().getDegrees();
      case PIGEON:
        return -pigeon.getAngle();
    }
  }
  
  public void printOffsets() {
    if (gyro_type == gyros.BNO55) {
      bno.readOffsets();
    }
  }
}
