// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.DriveGyro;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import static frc.robot.Constants.*;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drivetrain extends SubsystemBase {
  public static double dely = Units.inchesToMeters(0.5 * kSideWheelBase); // 0.2949 metters
  public static double delx = Units.inchesToMeters(0.5 * kFrontWheelBase);
  public static final double kMaxAcceleration = 2.0;
  public static final double kMaxVelocity = 3.0;
  public static final double kMaxAngularVelocity = 2.0 * Math.PI; // 1 rotation/s
  public static final double kMaxAngularAcceleration = Math.PI; // 1 rotations/s/s

  private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
	private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
	private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
	private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

  public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

  private final SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn,1);
  private final SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn,2);
  private final SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn,3);
  private final SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, 4);

  public static boolean m_field_oriented=false;

  private final SwerveModule[] modules={m_frontLeft,m_frontRight,m_backLeft,m_backRight};

  DigitalInput input = new DigitalInput(0);

  private final Field2d m_Field2d = new Field2d();
  Timer m_timer = new Timer();

  static boolean m_optimize=true;
  static boolean m_showtags=false;

  boolean m_resetting=false;

  public boolean getShowTags() {
    return m_showtags;
  }


  static int count = 0;
  DriveGyro m_gyro = new DriveGyro(DriveGyro.gyros.BNO55);
  double last_heading = 0; 
  SwerveModulePosition[] m_positions = {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition() };
      
  public Drivetrain() {
    SmartDashboard.putData("Field" , m_Field2d);
    SmartDashboard.putBoolean("Field Oriented" , m_field_oriented);
    SmartDashboard.putBoolean("Switch" , false);
    SmartDashboard.putBoolean("ShowTags", m_showtags);
   
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
        new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        0.3, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(false, false) // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        /*
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        */
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  private void resetPositions() {
    for(int i=0;i<modules.length;i++){
      modules[i].reset();
    }
    updatePositions();
  }

  private void updatePositions() {
    for(int i=0;i<modules.length;i++)
      m_positions[i] = modules[i].getPosition();
  }

  public void resetPose(Pose2d pose){
    System.out.println("resetPose working");
    m_poseEstimator.resetPosition(getRotation2d(), m_positions, pose);
  }

  private ChassisSpeeds getRobotRelativeSpeeds(){
    System.out.println("getRobotRelativeSpeeds working " + m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()));
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
  }

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      getRotation2d(),
      m_positions,
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // drive confidence stds
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // turn confidence stds

  public Rotation2d getRotation2d() {
    double angle =  m_gyro.getAngle();
    angle = unwrap(last_heading, angle);
    last_heading = angle;
    return Rotation2d.fromDegrees(angle);
  }

  public double getHeading(){
    return getRotation2d().getDegrees();
  }
  public void driveForwardAll(double dist) {
    for(int i=0;i<modules.length;i++)
      modules[i].driveForward(dist);
   
  }
  public void turnAroundAll(double dist) {
    for(int i=0;i<modules.length;i++)
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
    for(int i=0;i<modules.length;i++)
      modules[i].setDesiredState(swerveModuleStates[i]);
   
    updateOdometry();
  }

  private void driveRobotRelative(ChassisSpeeds speed){
    System.out.println("driveRobotRelative working");
    this.drive(speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond, false);
  }

  public static boolean isFieldOriented(){
    return m_field_oriented;
  }
  public void log() {
    SmartDashboard.putNumber("Gyro", getHeading());
    Pose2d pose=getPose();
    String s=String.format("X:%-2.1f Y:%-2.1f H:%-2.1f",
    pose.getX(),pose.getY(),pose.getRotation().getDegrees());
    SmartDashboard.putString("Pose", s);
    m_field_oriented=SmartDashboard.getBoolean("Field Oriented" , m_field_oriented);
    SmartDashboard.putBoolean("Switch" , input.get());
    m_showtags=SmartDashboard.getBoolean("ShowTags", m_showtags);

    for(int i=0;i<modules.length;i++)
      modules[i].log();
  }
     
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    updatePositions();
    m_poseEstimator.update(getRotation2d(), m_positions);
    m_Field2d.setRobotPose(getPose());
  }

  public void resetOdometry(Pose2d pose) {
		 m_gyro.reset();
    //resetPositions();
    m_poseEstimator.resetPosition(getRotation2d(),
        m_positions,
        pose);
    //last_heading=0;
	}
  public void resetOdometry() {
    m_gyro.reset();
    resetPositions();
    m_poseEstimator.resetPosition(getRotation2d(),
        m_positions,
        new Pose2d(0, 0, new Rotation2d()));
    last_heading=0;
  }
  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

    // removes heading discontinuity at 180 degrees
	public static double unwrap(double previous_angle, double new_angle) {
		double d = new_angle - previous_angle;
		d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
		return previous_angle + d;
	}
  public void reset() {
    // System.out.println("Drivetrain.reset before aligned="+wheelsAreAligned());
    // showWheelPositions();
    // m_optimize = false;
    // setOptimize(m_optimize);
    // m_timer.reset();
  }
 
  // reset wheels turn motor to starting position
  public void resetWheels(boolean begin){
    if(begin){
      m_resetting=true;
		  System.out.println("Drivetrain-ALIGNING_WHEELS");
    }
    for(int i=0;i<modules.length;i++){
      modules[i].resetWheel();
    }
	}
	
  // return true if all wheels are reset
  public boolean wheelsReset(){
    for(int i=0;i<modules.length;i++){
      if(!modules[i].wheelReset())
        return false;     
    }
    if(m_resetting)
      System.out.println("Drivetrain-WHEELS_ALIGNED");
    m_resetting = false;
		return true;
	}
  // another way to check if wheels are realighned
  public boolean wheelsAreAligned(){
    for(int i=0;i<modules.length;i++){
      for(int j=0;j<modules.length;j++){
        double h1=Math.toDegrees(modules[i].heading());
        double h2=Math.toDegrees(modules[j].heading());
        if(h1-h2>90)
          return false;
      }
    }
    return true;
  }
  public void showWheelPositions(){
    for(int i=0;i<modules.length;i++)
      modules[i].showWheelPosition();  
  }
  public void setOptimize(boolean b){
    for(int i=0;i<modules.length;i++)
      modules[i].setOptimize(b);
  }
   public boolean showTags() {
      return  m_showtags;
  }
 
  @Override
  public void periodic() {
    log();
  }
}
