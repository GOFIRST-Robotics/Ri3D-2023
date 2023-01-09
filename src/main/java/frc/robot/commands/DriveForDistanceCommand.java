// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForDistanceCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem;
  double initialDistance;
  double distance;
  double powerPct;

  /** Creates a new DriveForDistanceCommand. */
  public DriveForDistanceCommand(double distance, double powerPct) {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    this.distance = distance;
    this.powerPct = powerPct;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DRIVE FOR DISTANCE BEGAN");
    initialDistance = m_DriveSubsystem.getRightDistance();
    m_DriveSubsystem.drive(powerPct, powerPct);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DRIVING FOR DISTANCE");

    System.out.println("INITIAL DISTANCE: " + initialDistance);
    System.out.println("GOAL DISTANCE: " + (distance + initialDistance));
    System.out.println("CURRENT DISTANCE: " + m_DriveSubsystem.getRightDistance());
    System.out.println("POWER: " + powerPct);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVE FOR DISTANCE ENDED");
    m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_DriveSubsystem.getRightDistance() >= initialDistance + distance;
  }
}
