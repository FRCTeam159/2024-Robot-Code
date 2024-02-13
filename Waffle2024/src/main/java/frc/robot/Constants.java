// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 */
public final class Constants {
    
    public static final int kImageWidth = 640;
    public static final int kImageHeight = 480;

    public static final int kFl_Drive = 3;
    public static final int kFl_Turn = 8;

    public static final int kFr_Drive = 7;
    public static final int kFr_Turn = 6;

    public static final int kBr_Drive = 5;
    public static final int kBr_Turn = 1;

    public static final int kBl_Drive = 2;
    public static final int kBl_Turn = 4;

    public static final int kSpareSpark = 9;

    public static final double kPickup = 0;
    public static final double kSpeaker = 10;
    public static final double kAmp = 95;

    public static final double kArmGearRatio = 144.0 * 0.1;

}
