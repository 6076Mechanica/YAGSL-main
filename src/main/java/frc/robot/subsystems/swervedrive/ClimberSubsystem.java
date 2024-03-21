// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClimberSubsystem extends SubsystemBase {

  private PneumaticsControlModule m_ControlModule = new PneumaticsControlModule();
  private DoubleSolenoid m_doubleSolenoid = m_ControlModule.makeDoubleSolenoid(0, 1);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    //m_ControlModule.makeCompressor();
  }

  public void extend() {

  }

  public void retract() {
  
  } 

  public void toggle() {
    m_doubleSolenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
