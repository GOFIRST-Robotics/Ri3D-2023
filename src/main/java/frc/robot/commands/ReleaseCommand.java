// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;

// This command opens the grabber by extending the piston
public class ReleaseCommand extends CommandBase {

  GrabberSubsystem m_GrabberSubsystem;

  /** Creates a new ReleaseCommand. */
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}