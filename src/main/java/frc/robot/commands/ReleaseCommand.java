// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;

public class ReleaseCommand extends CommandBase {

  GrabberSubsystem m_GrabberSubsystem;

  /** Creates a new GrabCommand. */
  public ReleaseCommand() {
    m_GrabberSubsystem = Robot.m_grabberSubsystem;
  addRequirements(m_GrabberSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_GrabberSubsystem.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
