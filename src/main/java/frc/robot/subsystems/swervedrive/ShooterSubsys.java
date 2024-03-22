// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;


public class ShooterSubsys extends SubsystemBase {

  private final TalonFX m_talonfx = new TalonFX(motorPorts.shooterMotor);
  //private final XboxController driver = new XboxController(OperatorConstants.kDriverControllerPort);

  private double targetVelocity = 0.0;
  private final double Vtolerance = 20;

  /** Creates a new CustomPID. */
  public ShooterSubsys() {

    //Talon configuration:
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = falonConst.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = falonConst.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = falonConst.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = falonConst.kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = falonConst.kI; // no output for integrated error
    slot0Configs.kD = falonConst.kD; // no output for error derivative

    // Set up Motion Magic Velocity settings.
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = falonConst.kFalconAccel; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_talonfx.getConfigurator().apply(talonFXConfigs);    
    m_talonfx.setNeutralMode(NeutralModeValue.Coast);
    
  }

  @Override
  public void periodic() {
    var speed = m_talonfx.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Talon reported V", speed);
    SmartDashboard.putBoolean("Talon ready", ready());
    //m_talonfx.set(driver.getRightTriggerAxis());
  }
  
  public void SpinUp() {
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    m_talonfx.setControl(m_request.withVelocity(targetVelocity));
  }

  public void setSpitSpeed() {
    targetVelocity = 0.4;
  }

  public void setShootSpeed() {
    targetVelocity = 0.95;
  }

  public void SpinDown() {
    m_talonfx.disable();
  }

  public Boolean ready() {
    var velocity = m_talonfx.getVelocity().getValueAsDouble();
    if (velocity > 85) {
      return true;
    } else {
      return false;
    }
  }

  public void spitSlow() {
    m_talonfx.set(0.25);
  }

  public void spitMed() {
    m_talonfx.set(0.5);
  }

  public void spit() {
    m_talonfx.set(targetVelocity);
  }

  public void spitsuperFast() {
    m_talonfx.set(0.95);
  }

  public void Stop() {
    m_talonfx.set(0.0);
  }

  public void eject() {
    m_talonfx.set(0.3);
  }


}
