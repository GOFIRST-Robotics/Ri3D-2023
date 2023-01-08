// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class StopDrivetrainCommand extends CommandBase {
  private DriveSubsystem m_DriveSubsystem;

  public StopDrivetrainCommand() {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // Command will never finish
  }
}