// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.sensors.DriveGyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drivetrain extends SubsystemBase {
  static public final double kDriveGearRatio = 6.67; // MK4i drive (standard)
  static public final double kTurnGearRatio = 30; // MK4i turn (all)

  public static final double kWheelRadius = 2;
  public static final double kDistPerRot = (Units.inchesToMeters(kWheelRadius) * 2 * Math.PI) / kDriveGearRatio;
  public static final double kRadiansPerRot = Math.PI * 2 / kTurnGearRatio;

  public static final double kRobotLength = Units.inchesToMeters(24); // Waffle side length

  public static final double kFrontWheelBase = Units.inchesToMeters(19); // distance bewteen front wheels
  public static final double kSideWheelBase = Units.inchesToMeters(19); // distance beteen side wheels
  public static final double kTrackRadius = 0.5
      * Math.sqrt(kFrontWheelBase * kFrontWheelBase + kSideWheelBase * kSideWheelBase);

  public static final double kMaxVelocity = 1.0;
  public static final double kMaxAcceleration = 0.5;
  public static final double kMaxAngularVelocity = Math.toRadians(720); // radians/s
  public static final double kMaxAngularAcceleration = Math.toRadians(360); // radians/s/s

  public static double dely = 0.5 * kSideWheelBase; // 0.2949 metters
  public static double delx = 0.5 * kFrontWheelBase;

  private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
  private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
  private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
  private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);
  public static String chnlnames[] = { "FL", "FR", "BR", "BL" };

  private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn, 1);
  private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn, 2);
  private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn, 3);
  private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, 4);

  public static boolean m_field_oriented = false;
  boolean m_show_module_status = false;

  private final SwerveModule[] modules = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveModulePosition[] m_positions = {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition() };

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), m_positions, new Pose2d());

  //DigitalInput input = new DigitalInput(0);

  private final Field2d m_Field2d = new Field2d();

  Pose2d m_pose;
  Timer m_timer = new Timer();

  boolean m_resetting = false;

  static int count = 0;
  //DriveGyro m_gyro = new DriveGyro(DriveGyro.gyros.NAVX); // moved to arm
  DriveGyro m_gyro=new DriveGyro(DriveGyro.gyros.FRC450);   // seems to work now (?)

  double last_heading = 0;

  public Drivetrain() {
    m_frontLeft.setDriveInverted(false);
    m_backLeft.setDriveInverted(false);

    m_frontRight.setDriveInverted(true);
    m_backRight.setDriveInverted(true);

    resetOdometry();

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(7.0, 0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            /* Units.inchesToMeters(12.1) */0.3, // Drive base radius in meters. Distance from robot center to furthest
                                                 // module.
            new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          /*
           * var alliance = DriverStation.getAlliance();
           * if (alliance.isPresent()) {
           * return alliance.get() == DriverStation.Alliance.Red;
           * }
           */
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
  }
  private void driveRobotRelative(ChassisSpeeds speed) {
    this.drive(speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond, false);
  }

  private void resetPositions() {
    for (int i = 0; i < modules.length; i++) 
      modules[i].reset();
    updatePositions();
  }

  private void updatePositions() {
    for (int i = 0; i < modules.length; i++)
      m_positions[i] = modules[i].getPosition();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), m_positions, pose);
  }

  public Rotation2d getRotation2d() {
    double angle = m_gyro.getAngle();
    angle = unwrap(last_heading, angle);
    last_heading = angle;
    return Rotation2d.fromDegrees(angle);
  }

  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public void driveForwardAll(double dist) {
    for (int i = 0; i < modules.length; i++)
      modules[i].driveForward(dist);
  }

  public void turnAroundAll(double dist) {
    for (int i = 0; i < modules.length; i++)
      modules[i].turnAround(dist);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
    for (int i = 0; i < modules.length; i++)
      modules[i].setDesiredState(swerveModuleStates[i]);

    updateOdometry();
  }
  
  public static boolean isFieldOriented() {
    return m_field_oriented;
  }

  public void log() {
    SmartDashboard.putNumber("Gyro", getHeading());
    Pose2d pose = getPose();
    String s = String.format("X:%-2.1f Y:%-2.1f H:%-2.1f",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    SmartDashboard.putString("Pose", s);
    m_field_oriented = SmartDashboard.getBoolean("Field Oriented", m_field_oriented);
    //SmartDashboard.putBoolean("Switch", input.get()
    if (m_show_module_status) {
      for (int i = 0; i < modules.length; i++)
        modules[i].log();
    }

    SmartDashboard.putString("Status", Robot.status);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    updatePositions();
    m_pose = m_odometry.update(getRotation2d(), m_positions);
    //m_Field2d.setRobotPose(getPose());
  }

  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    // resetPositions();
    m_odometry.resetPosition(getRotation2d(), m_positions, pose);
    // last_heading=0;
  }

  public void resetOdometry() {
    m_gyro.reset();
    resetPositions();
    m_odometry.resetPosition(getRotation2d(), m_positions, new Pose2d(0, 0, new Rotation2d()));
    last_heading = 0;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getVelocity() {
    double left = 0.5 * (m_frontLeft.getVelocity() + m_backLeft.getVelocity());
    double right = 0.5 * (m_frontRight.getVelocity() + m_backRight.getVelocity());
    return 0.5 * (left + right);
  }

  public double getDistance() {
    double left = 0.5 * (m_frontLeft.getDistance() + m_backLeft.getDistance());
    double right = 0.5 * (m_frontRight.getDistance() + m_backRight.getDistance());
    return 0.5 * (left + right);
  }

  // removes heading discontinuity at 180 degrees
  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }

  public void reset() {
    m_gyro.reset();
    resetPositions();
    last_heading = 0;
  }

  // reset wheels turn motor to starting position
  public void resetWheels(boolean begin) {
    if (begin) {
      m_resetting = true;
      System.out.println("Drivetrain-ALIGNING_WHEELS");
    }
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetWheel();
    }
  }

  // return true if all wheels are reset
  public boolean wheelsReset() {
    for (int i = 0; i < modules.length; i++) {
      if (!modules[i].wheelReset())
        return false;
    }
    if (m_resetting)
      System.out.println("Drivetrain-WHEELS_ALIGNED");
    m_resetting = false;
    return true;
  }

  @Override
  public void periodic() {
    updateOdometry();
    log();
  }
}
