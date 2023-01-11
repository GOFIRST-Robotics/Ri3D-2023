// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

// This command drives a specified number of meters
public class DriveForDistanceCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem;
  double initialDistance;
  double distance;
  double percentPower;

  /** Creates a new DriveForDistanceCommand. */
  public DriveForDistanceCommand(double distance, double percentPower) {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    this.distance = distance;
    this.percentPower = percentPower;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialDistance = m_DriveSubsystem.getRightDistance();
    System.out.println("INITIAL DISTANCE: " + initialDistance);
    m_DriveSubsystem.drive(percentPower, percentPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Print statements for debugging
    System.out.println("GOAL DISTANCE: " + (distance + initialDistance));
    System.out.println("CURRENT DISTANCE: " + m_DriveSubsystem.getRightDistance());
    System.out.println("POWER: " + percentPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop(); // Stop the rivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_DriveSubsystem.getRightDistance() >= initialDistance + distance; // End the command when we have reached our goal
  }
}