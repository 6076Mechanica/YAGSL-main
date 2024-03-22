// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeSubsys extends SubsystemBase {

  //CANSparkMax FeederNeo = new CANSparkMax(motorPorts.feedMotor, MotorType.kBrushless);
  private CANSparkMax nSparkMax = new CANSparkMax(motorPorts.intakeMotor, MotorType.kBrushless);
  private AnalogInput m_sensor = new AnalogInput(0);

  private final double kDefaultSpeed = 0.75;
  private final double kConveyerSpeed = 0.15;
  private final double kFeedSpeed = 0.5;
  public boolean isReady;

  public IntakeSubsys() {
    nSparkMax.setIdleMode(IdleMode.kCoast);
    nSparkMax.setInverted(true);
    //FeederNeo.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ready?: ", isReady);
  }

  public Double getVelocity() {
    //DoubleSupplier sup = () -> nSparkMax.getEncoder().getVelocity();
    return nSparkMax.getEncoder().getVelocity();
  }

  public void intake() {
    nSparkMax.set(kDefaultSpeed);
  }

  public void convey() {
    if (!isReady) {
    nSparkMax.set(0.75);
    } else {
      // Do nothing.
    }
  }

  public void feed(double speed) {
    double feedSpeed = speed/2;
    //FeederNeo.set(feedSpeed);
    //isReady = false;
  }

  public void setSpeed(double speed) {
    nSparkMax.set(speed);
  }

  public void stop() {
    nSparkMax.set(0.0);
  }

  public Boolean isStaged() {
    if(m_sensor.getVoltage() < 1.0) {
      return false;
    } else if (m_sensor.getVoltage() > 1.0) {
      return true;
    }else {
      return false;
    }
  }

  public void setReady() {
    isReady = true;
  }

  public void setUnready() {
    isReady = false;
  }

  public Boolean isReady() {
    return isReady;
  }

  public void eject() {
    nSparkMax.set(-0.3);
  }

}
