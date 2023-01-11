// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

// Uses PID control to align to the target angle using gyroscope feedback
public class GyroTurnToAngleCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem; // drivetrain subsystem
  double degreesToTurn; // the number of degrees we wish to turn
  double error; // How "incorrect" the current angle of the robot is as its moving
  double targetAngle; // targetAngle = initial angle + degreesToTurn

  /** Turns to an angle relative to the current angle using the gyro */
  public GyroTurnToAngleCommand(double degreesToTurn) {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
    this.degreesToTurn = degreesToTurn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetAngle = degreesToTurn + m_DriveSubsystem.getAngle();
    System.out.println("CURRENT ANGLE:" + m_DriveSubsystem.getAngle());
    System.out.println("TARGET ANGLE:" + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - m_DriveSubsystem.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which gets our current angle from the robot

    double value = error * Constants.GYRO_TURN_KP; // Multiply by scaling factor kp to determine motor percent power between 0 and 100 percent
    if (Math.abs(value) > 0.75) { // Maximum drive value we want
      value = Math.copySign(0.75, value);
    }
    if (Math.abs(value) < 0.15) { // Minimum drive value we want
      value = Math.copySign(0.15, value);
    }
    // Print statements for debugging //
    System.out.println("error:" + error);
    System.out.println("value:" + value);

    m_DriveSubsystem.drive(value, -value); // drive with the calculated values
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop(); // Stop the drivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.DRIVE_TURNING_THRESHOLD_DEGREES; // End the command when we are within the specified threshold of our target
  }
}