// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimberSubsystem extends SubsystemBase {

  private DoubleSolenoid m_doubleSolenoid;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    try (PneumaticsControlModule m_ControlModule = new PneumaticsControlModule()) {
      m_doubleSolenoid = m_ControlModule.makeDoubleSolenoid(0, 1);
      m_ControlModule.makeCompressor();
    } 
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggle() {
    m_doubleSolenoid.toggle();
  }

  public void extend() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
