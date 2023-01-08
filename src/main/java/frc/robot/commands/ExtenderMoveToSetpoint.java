// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderMoveToSetpoint extends CommandBase {
  /** Creates a new Extender Command. */
  private ExtenderSubsystem m_ExtenderSubsystem;
  private double goalPos;
  private double error;
  private int setPoint;

  public ExtenderMoveToSetpoint(int setPoint) {
    // Use addRequirements() here to declare subsystem dependencies. 
    m_ExtenderSubsystem = Robot.m_extenderSubsystem;
    addRequirements(m_ExtenderSubsystem);
    this.setPoint = setPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ExtenderSubsystem.currentSetpoint = setPoint;

    switch (setPoint) {
      case 0: goalPos = Constants.EXTENDER_SETPOINT_INTAKE;
      case 1: goalPos = Constants.EXTENDER_SETPOINT_1;
      case 2: goalPos = Constants.EXTENDER_SETPOINT_2;
      case 3: goalPos = Constants.EXTENDER_SETPOINT_3;
      case 4: goalPos = Constants.EXTENDER_SETPOINT_4;
    }
  }

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