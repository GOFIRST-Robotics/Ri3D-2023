// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ExtenderSubsystem;

// FIXME: As of now, this command does NOT currently work how we want it to; use this code with caution

// ExtenderMoveToSetpointCommand is a command that moves the extender to a setpoint 
// This is the default command of the Extender subsystem
public class ExtenderControlCommand extends CommandBase {

  /** Command to control the robot extender for the arm */
  private ExtenderSubsystem m_ExtenderSubsystem;
  private double goalPos;
  private double error;

  public ExtenderControlCommand() {
    m_ExtenderSubsystem = Robot.m_extenderSubsystem;
    addRequirements(m_ExtenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only run PID on the extender if we want to //
    if (Robot.toggleExtenderPID.getSelected()) {
      System.out.println("SETPOINT" + m_ExtenderSubsystem.getCurrentSetPoint());
      switch (m_ExtenderSubsystem.getCurrentSetPoint()) {
        case 0: goalPos = Constants.EXTENDER_SETPOINT_INTAKE;
                break;
        case 1: goalPos = Constants.EXTENDER_SETPOINT_1;
                break;
        case 2: goalPos = Constants.EXTENDER_SETPOINT_2;
                break;
        case 3: goalPos = Constants.EXTENDER_SETPOINT_3;
                break;
        case 4: goalPos = Constants.EXTENDER_SETPOINT_4;
                break;
      }
      System.out.println("GOAL" + goalPos);
      error = goalPos - m_ExtenderSubsystem.getEncoderPosition();
      if (Math.abs(error) > Constants.EXTENDER_TOLERANCE) {
        double power = Constants.EXTENDER_KP * error;
        if (Math.abs(power) > Constants.EXTENDER_POWER) {
          power = Math.copySign(Constants.EXTENDER_POWER, power);
        }
        if (Math.abs(power) < 0.2) {
          power = Math.copySign(0.2, power);
        }
        m_ExtenderSubsystem.setPower(Constants.EXTENDER_KP * error);
      } else {
        m_ExtenderSubsystem.stop();
      }
    } 
    
    // Manual Control of the Extender //
    else {
      if (Robot.controller.getRawButton(Constants.RIGHT_TRIGGER_AXIS)) {
          m_ExtenderSubsystem.setPower(Constants.EXTENDER_POWER);
      } else if (Robot.controller.getRawButton(Constants.LEFT_TRIGGER_AXIS)) {
        m_ExtenderSubsystem.setPower(-Constants.EXTENDER_POWER);
      } else {
        m_ExtenderSubsystem.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ExtenderSubsystem.stop(); // Stop the extender motors
  }

  public boolean isFinished() {
    return false; // This command will never end (usually only ideal for always-running default commands)
  }
}