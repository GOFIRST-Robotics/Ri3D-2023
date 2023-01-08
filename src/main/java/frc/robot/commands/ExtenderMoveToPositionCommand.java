// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderMoveToPositionCommand extends CommandBase {
  /** Creates a new Extender Command. */
  private ExtenderSubsystem m_ExtenderSubsystem;
  private double goalPos;
  private double error;

  public ExtenderMoveToPositionCommand(double goalPos) {
    // Use addRequirements() here to declare subsystem dependencies. 
    m_ExtenderSubsystem = Robot.m_extenderSubsystem;
    addRequirements(m_ExtenderSubsystem);
    this.goalPos = goalPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = goalPos - m_ExtenderSubsystem.getEncoderPosition();
    m_ExtenderSubsystem.setPower(Constants.EXTENDER_KP * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ExtenderSubsystem.setPower(0.0);
  }

  public boolean isFinished() {
    return Math.abs(error) < 50;
  }
}