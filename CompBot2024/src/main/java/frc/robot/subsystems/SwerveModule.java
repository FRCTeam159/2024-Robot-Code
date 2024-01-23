// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANcoder m_turningEncoder;
  private final RelativeEncoder m_driveEncoder;
 
  public static boolean debug = true;
  String name;

  int cnt = 0;

  boolean m_inverted = false;
  boolean m_optimize = true;
  // SET calibrate_offsets=false TO CALIBRATE CANCODER OFFSETS
  // - READ VALUES FROM SMART DASHBOARD
  // - SET OFFSETS ON CONSTANTS
  // - THEN SET  calibrate_offsets=true
  static boolean calibrate_offsets=false;
  static double RotationsToRadians=2*Math.PI;

  // PID controllers for drive and steer motors
  private final PIDController m_drivePIDController = new PIDController(2, 0, 0);
  private final PIDController m_turningPIDController = new PIDController(
      0.5,
      0,
      0
      // new TrapezoidProfile.Constraints(
      //     kMaxAngularSpeed, kMaxAngularAcceleration)
      );

  //private final PIDController m_turningPIDController = new PIDController(7, 0, 0);

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoder
   */

  private int m_id;
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoder,
      double turningEncoderOffset,
      int id) {

    m_id = id;

    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

    name = Drivetrain.chnlnames[m_id - 1];
    SmartDashboard.putString(name, "Working");

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(kDistPerRot); // inches to meters
    m_driveEncoder.setVelocityConversionFactor(kDistPerRot / 60); // convert RPM to meters per second

    // Set the distance (in this case, angle) in radians per pulse for the turning
    //m_turningEncoder = m_turningMotor.getEncoder();
     m_turningEncoder = new CANcoder(turningEncoder);

     CANcoderConfiguration config = new CANcoderConfiguration();
     // set units of the CANCoder to radians, with velocity being radians per second
    // config.sensorCoefficient = 2 * Math.PI / kEncoderResolution; // 4096 for CANcoder
    // config.unitString = "rad";
    // config.sensorTimeBase = SensorTimeBase.PerSecond; // set timebase to seconds
     config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // should avoid discontinuity at 0 degrees or
                                                                           // 360
     // config.absoluteSensorRange=AbsoluteSensorRange.Unsigned_0_to_360;
     if(calibrate_offsets){
      //config.initializationStrategy = SensorInitializationStrategy.BootToZero;
      config.MagnetSensor.MagnetOffset  = 0;     
    }
    else{
      //config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
      config.MagnetSensor.MagnetOffset = turningEncoderOffset; // now in rotations
    }
    m_turningEncoder.getConfigurator().apply(config);
    
    //TODO check example and see if this is there: 
    m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
  }

  public double getRotations(){
    return m_driveEncoder.getPosition()/m_driveEncoder.getPositionConversionFactor();
  }
  public void reset(){
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
    System.out.println(name + " reset");
    cnt=0;
  }
  void setOptimize(boolean t){
    m_optimize=t;
  }
  
  public double heading(){
    StatusSignal<Double> val=m_turningEncoder.getPosition();
    return RotationsToRadians*val.getValue(); // return radians for rotations
  }
  
  public double cummulativeAngle(){
    return heading();
  }
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(cummulativeAngle());
  }

  public void showWheelPosition(){
    System.out.format("%s %-3.1f\n",name,Math.toDegrees(heading()));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getVelocity(), getRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDistance(){
    return m_driveEncoder.getPosition();
  }

  public double getVelocity() {
      return m_driveEncoder.getVelocity();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state;
      state= SwerveModuleState.optimize(desiredState, getRotation2d());
    //System.out.println("optimize = " + m_optimize);
    double velocity=getVelocity();
    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(velocity, state.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    double turn_angle=getRotation2d().getRadians();

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.calculate(turn_angle, state.angle.getRadians());
    double turnFeedforward = 0; //-m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);


    double set_drive=driveOutput +driveFeedforward;
    double set_turn=turnOutput+turnFeedforward;

    //System.out.println(set_drive);
    //System.out.println(set_turn);

    driveForward(set_drive);
    m_turningMotor.set(set_turn);
    
    if(debug){
      String s = String.format("Vel %-2.2f(%-2.2f) -> %-2.2f Angle %-3.3f(%-2.3f) -> %-2.3f\n", 
      velocity,state.speedMetersPerSecond,set_drive,Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn); 
      SmartDashboard.putString(name, s);
    
    }   
  }

  public void log() {
    String s = String.format("Drive:%-1.3f m Angle:%-4.1f Rotations:%-4.2f\n", 
    getDistance(), getRotation2d().getDegrees(),getRotations());
    SmartDashboard.putString(name, s);
    
  }

  public boolean isInverted(){
    return m_inverted;
  }

  public void setDriveInverted(boolean b){
    m_inverted = b;
  m_driveMotor.setInverted(b);
  }
  
  public void driveForward(double dist) {
   //dist = m_inverted? -dist: dist;
    m_driveMotor.setVoltage(dist);
  }
  
  public void turnAround(double dist) {
    m_turningMotor.setVoltage(dist);
  }
 
  @Override
  public void periodic() {
    if(debug)
      log();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
