// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    static public final double kDriveGearRatio=8.14;  // MK4i drive (standard)
    static public final double kTurnGearRatio=21.429; // MK4i turn (all)
    public static final double kFrontWheelBase = 24; // inches bewteen front wheels
	public static final double kSideWheelBase = 24;  // inches beteen side wheels
    public static double dely = Units.inchesToMeters(0.5 * kSideWheelBase);
    public static double delx = Units.inchesToMeters(0.5 * kFrontWheelBase);
    
    public static final double kMaxAcceleration = 1.0;  // m/s/s
    public static final double kMaxVelocity = 4;  // m/s
    public static final double kMaxAngularAcceleration = 2*Math.PI; // 1 rotations/s/s
    public static final double kRadius = Math.sqrt(Math.pow(delx, 2) + Math.pow(dely, 2));
    public static final double kMaxAngularVelocity = 28; // kMaxVelocity/kRadius; // radians/s
  
    public static final double kWheelRadius = 2;
    public static final int kEncoderResolution = 42;
    public static final double kDistPerRot =(Units.inchesToMeters(kWheelRadius)* 2*Math.PI)/kDriveGearRatio;
    public static final double kRadiansPerRot = Math.PI * 2/kTurnGearRatio;

    public static final double kFrontRightOffset/*9*/ = 0.403076;
    public static final double kFrontLeftOffset/*10*/ = -0.405518;
    public static final double kBackLeftOffset/*11*/ = 0.268555;
    public static final double kBackRightOffset/*12*/ = 0.156730;

    public static final int kImageWidth = 640;
    public static final int kImageHeight = 480;

    public static final int kFl_Drive = 3;
    public static final int kFl_Turn = 7;
    public static final int kFl_Encoder = 10;
    public static final double kFl_Offset = kFrontLeftOffset;  // now fractional rotation ?

    public static final int kFr_Drive = 4;
    public static final int kFr_Turn = 8;
    public static final int kFr_Encoder = 9;
    public static final double kFr_Offset = kFrontRightOffset;

    public static final int kBr_Drive = 2;
    public static final int kBr_Turn = 6;
    public static final int kBr_Encoder = 12;
    public static final double kBr_Offset = kBackRightOffset; 

    public static final int kBl_Drive = 1;
    public static final int kBl_Turn = 5;
    public static final int kBl_Encoder = 11;
    public static final double kBl_Offset = kBackLeftOffset; 

    public static final int kShoulderMotor1 = 13; 
    public static final int kShoulderMotor2 = 14;
    public static final int kIntakeMotor = 18;
    public static final int kShooterMotor1 = 15;
    public static final int kShooterMotor2 = 16;

    public static final int kPigeonCanId = 30;

    // arm angle settings
    public static final double kPickup = 10;
    public static final double kSpeaker = 20;
    public static final double kAmp = 95;
}
