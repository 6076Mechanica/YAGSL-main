// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.ShooterSubsys;

public class ShootCMD extends Command {

  ShooterSubsys m_shooter;

  public ShootCMD(ShooterSubsys subsys) {
    m_shooter = subsys;
  }

  public Command shoot() {
    return Commands.runOnce(m_shooter::SpinUp);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
