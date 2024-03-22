// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsys extends SubsystemBase {

  private final CANSparkMax FeederNeo = new CANSparkMax(motorPorts.feedMotor, MotorType.kBrushless);
  //private final AnalogInput m_sensor = new AnalogInput(0);
  private final DigitalInput m_beam = new DigitalInput(0);

  private final double kConveyerSpeed = 0.15;
  private final double kFeedSpeed = 0.5;

  public FeederSubsys() {
    FeederNeo.setInverted(true);
    FeederNeo.setIdleMode(IdleMode.kBrake);
  }

  public void feed() {
    FeederNeo.set(kFeedSpeed);
  }

  public void stopFeed() {
    FeederNeo.set(0);
  }

  public boolean isStaged() {
    if (m_beam.get()) {
      return false;
    } else if (!m_beam.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isEmpty() {
    return m_beam.get();
  }

  public void eject() {
    FeederNeo.set(-0.3);
  }

  // public Boolean isStaged() {
  //   if(m_sensor.getVoltage() < 1.0) {
  //     return false;
  //   } else if (m_sensor.getVoltage() > 1.0) {
  //     return true;
  //   }else {
  //     return false;
  //   }
  // }

  // public Boolean isEmpty() {
  //   if(m_sensor.getVoltage() < 1.0) {
  //     return true;
  //   } else if (m_sensor.getVoltage() > 1.0) {
  //     return false;
  //   }else {
  //     return true;
  //   }
  // }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("feederEmpty?", isEmpty());
  }

}
