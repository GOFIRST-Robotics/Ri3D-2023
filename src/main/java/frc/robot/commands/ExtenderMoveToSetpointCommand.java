// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ExtenderSubsystem;

// ExtenderMoveToSetpointCommand is a command that moves the extender to a setpoint 
// This is the default command of the Extender subsystem
public class ExtenderMoveToSetpointCommand extends CommandBase {

  /** Command to control the robot extender for the arm */
  private ExtenderSubsystem m_ExtenderSubsystem;
  private double goalPos;
  private double error;

  public ExtenderMoveToSetpointCommand() {
    m_ExtenderSubsystem = Robot.m_extenderSubsystem;
    addRequirements(m_ExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_ExtenderSubsystem.getCurrentSetPoint()) {
      case 0: goalPos = Constants.EXTENDER_SETPOINT_INTAKE;
      case 1: goalPos = Constants.EXTENDER_SETPOINT_1;
      case 2: goalPos = Constants.EXTENDER_SETPOINT_2;
      case 3: goalPos = Constants.EXTENDER_SETPOINT_3;
      case 4: goalPos = Constants.EXTENDER_SETPOINT_4;
    }

    error = goalPos - m_ExtenderSubsystem.getEncoderPosition();
    if (error > Constants.EXTENDER_TOLERANCE) {
      m_ExtenderSubsystem.setPower(Constants.EXTENDER_KP * error);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ExtenderSubsystem.stop();
  }

  public boolean isFinished() {
    return false;
  }
}