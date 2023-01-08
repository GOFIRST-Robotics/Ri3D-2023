// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;

public class ActuateGrabber extends CommandBase {
  /** Creates a new ActuateClaw. */
  private GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();

  public ActuateGrabber(double powerPct) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_GrabberSubsystem.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_GrabberSubsystem.retract();
  }
}