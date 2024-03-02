// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.kIntakeMotor;
import static frc.robot.Constants.kShooterMotor1;
import static frc.robot.Constants.kShooterMotor2;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import utils.Averager;


public class IntakeShooter extends SubsystemBase {
  double m_shoot_max_speed = 4800; // Old: 4800;
  double m_shoot_initial_speed = 3500; // Old: 3500;  // units: RPM
  double m_shoot_amp_speed = 3000;
  double m_shoot_target_speed = m_shoot_initial_speed;

  private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.01, 1);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.01, 1/m_shoot_max_speed);

  private final PIDController m_shooter1PIDController = new PIDController(0.4/m_shoot_max_speed, 0, 0);
  private final PIDController m_shooter2PIDController = new PIDController(0.4/m_shoot_max_speed, 0, 0);

  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_shooterMotor1;
  private final CANSparkMax m_shooterMotor2;

  DigitalInput noteSensor1 = new DigitalInput(1);
  DigitalInput noteSensor2 = new DigitalInput(2);
  private static final String name = "IntakeShooter";
  public boolean m_shoot = false;
  public boolean m_intake = false;
  public boolean m_push = false;
  boolean m_noteHasReachedShooter = false;
  public boolean m_hasNote = false;
  Averager sensor1_averager=new Averager(5);
  Averager sensor2_averager=new Averager(5);
  double m_notAtIntakeAve=0;
  double m_notAtShooterAve=0;

  /** Creates a new IntakeShooter. */
  public IntakeShooter() {
    m_intakeMotor = new CANSparkMax(kIntakeMotor, CANSparkLowLevel.MotorType.kBrushed);
    m_intakeMotor.setSmartCurrentLimit(38);
    m_intakeMotor.setInverted(true);
    m_shooterMotor1 = new CANSparkMax(kShooterMotor1, CANSparkLowLevel.MotorType.kBrushless);
    m_shooterMotor1.setInverted(true);
    m_shooterMotor2 = new CANSparkMax(kShooterMotor2, CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
    runIntake();
  }

  private void log() {
    SmartDashboard.putBoolean(name + "Sensor_1", noteAtIntake());
    SmartDashboard.putBoolean(name + "Sensor_2", noteAtShooter());
    SmartDashboard.putNumber(name + "Shooter1Speed", shooter1Speed());
    SmartDashboard.putNumber(name + "Shooter2Speed", shooter2Speed());
  }

  private void runIntake() {
    // These are "normally closed" so we invert them to see if they're sensing something
    double intakeCommand = 0;
    double val=noteSensor1.get()?0:1;
    m_notAtIntakeAve=sensor1_averager.getAve(val);
    val=noteSensor2.get()?0:1;
    m_notAtIntakeAve=sensor2_averager.getAve(val);

    if(m_push)
      intakeCommand = 1; // fire the shot
    else if (m_intake) { // pickup or carry a note
      // when attempting to intake
      if (noteAtIntake() && noteAtShooter()) {
        // intake should run backward (out) if note sensor 2 is triggered
          intakeCommand = -0.2;
          m_noteHasReachedShooter = true;
      } else if (noteAtIntake() && !noteAtShooter() && m_noteHasReachedShooter) {
        // intake should stop if only note sensor 1 is triggered
          intakeCommand = 0;
          m_hasNote = true;
      } else {
        // intake should run forward (in) if there is no note
          intakeCommand = 1;
      }
    }
    m_intakeMotor.set(intakeCommand);
    // Override these for shooting mode
    if (m_shoot) {
      // Lower shooting speed if arm is at amp angle
      if (Arm.lowSpeed) {
        m_shoot_target_speed = m_shoot_amp_speed;
      } else {
        m_shoot_target_speed = m_shoot_initial_speed;
      } 

      // motor 1
      double command = m_shooterFeedforward.calculate(m_shoot_target_speed) + 
      m_shooter1PIDController.calculate(shooter1Speed(), m_shoot_target_speed);
      m_shooterMotor1.set(command);
      // motor 2
      command = m_shooterFeedforward.calculate(m_shoot_target_speed) + 
      m_shooter2PIDController.calculate(shooter2Speed(), m_shoot_target_speed);
      m_shooterMotor2.set(command);
    }
    else{
      m_shooterMotor1.set(0);
      m_shooterMotor2.set(0);
    }
  }

  public void setShooterOn() {
    m_shoot = true;
  }
  public void setShooterOff() {
    m_shoot = false;
  }
  public void setIntakeOn() {
    m_intake = true;
    m_noteHasReachedShooter = false;
    m_hasNote = false;
  }
  public void setIntakeOff() {
    m_intake = false;
    m_noteHasReachedShooter = false;
    m_hasNote = false;
  }
  public void setPushOn() {
    m_push = true;
  }
  public void setPushOff() {
    m_push = false;
  }
  public boolean noteAtIntake(){
    //return !noteSensor1.get();
    return m_notAtIntakeAve>=0.5?true:false;
  }
  public boolean noteAtShooter(){
    //return !noteSensor2.get();
    return m_notAtShooterAve>=0.5?true:false;
  }

  public double shooter1Speed(){
    return m_shooterMotor1.getEncoder().getVelocity();
  }

  public double shooter2Speed(){
    return m_shooterMotor2.getEncoder().getVelocity();
  }
  public boolean atTargetSpeed(){
    // Check if we're at 95% of our target
    double speedTargetMargin = m_shoot_target_speed * 0.95;
    return shooter1Speed() >= speedTargetMargin && shooter2Speed() >= speedTargetMargin;
  }
}
