// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swervedrive.IntakeSubsys;

import java.util.function.DoubleSupplier;

public class RunGobbler extends Command {
  
  IntakeSubsys m_gobbler;

  DoubleSupplier Input;

  public RunGobbler(DoubleSupplier input, IntakeSubsys subsys) {
    m_gobbler = subsys;
    addRequirements(subsys);
    this.Input = input;
  }

  public RunGobbler(IntakeSubsys subsys) {
    m_gobbler = subsys;
    addRequirements(subsys);
  }

  public RunGobbler() {

  }

  public Command RunFor5() {
    return Commands.run(m_gobbler::intake).withTimeout(5);
  }

  public Command Stop() {
    return Commands.runOnce(m_gobbler::stop);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
