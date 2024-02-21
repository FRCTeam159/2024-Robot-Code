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


public class IntakeShooter extends SubsystemBase {

  private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0.01, 1);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(0.01, 1);

  private final PIDController m_shooterPIDController = new PIDController(0.01, 0, 0);

  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_shooterMotor1;
  private final CANSparkMax m_shooterMotor2;

  DigitalInput noteSensor1 = new DigitalInput(1);
  DigitalInput noteSensor2 = new DigitalInput(2);
  private static final String name = "IntakeShooter";
  public boolean shoot = false;
  public boolean intake = false;


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

  public void log() {
    SmartDashboard.putBoolean(name + "Sensor_1", !noteSensor1.get());
    SmartDashboard.putBoolean(name + "Sensor_2", !noteSensor2.get());
  }

  private void runIntake() {
    // These are "normally closed" so we invert them to see if they're sensing something
    boolean sensor1State = !noteSensor1.get(); 
    boolean sensor2State = !noteSensor2.get();
    double intakeCommand = 0;

    if (intake) {
      // when attempting to intake
      if (!sensor1State) {
        // intake should run forward (in) if there is no note
          intakeCommand = 1;
      } else if (sensor1State && sensor2State) {
        // intake should run backward (out) if note sensor 2 is triggered
          intakeCommand = -0.2;
      } else if (sensor1State && !sensor2State) {
        // intake should stop if only note sensor 1 is triggered
          intakeCommand = 0;
      }
    }

    // Override these for shooting mode
    if (shoot) {
      intakeCommand = 0.5;
    }
    m_intakeMotor.set(intakeCommand);
  }

  public void intake() {
    intake = true;
  }

  public void stopIntake() {
    intake = false;
  }

  public void shoot() {
    shoot = true;
    m_shooterMotor1.set(1);
    m_shooterMotor2.set(1);
  }
  
  public void stopShoot() {
    shoot = false;
    m_shooterMotor1.set(0);
    m_shooterMotor2.set(0);
  }
}
