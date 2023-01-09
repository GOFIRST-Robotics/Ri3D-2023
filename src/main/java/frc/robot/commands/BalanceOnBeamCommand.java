// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

// This command takes the joystick inputs and demands that the drivetrain follow them
public class BalanceOnBeamCommand extends CommandBase {

  private DriveSubsystem m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double previousAngle;
  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public BalanceOnBeamCommand() {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment below to simulate gyro axis with controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    currentAngle = m_DriveSubsystem.getPitch();

    error = Constants.BEAM_BALANCED_ANGLE_DEGREES - currentAngle;
    double changeInAngle = currentAngle - previousAngle;
    previousAngle = currentAngle;

    // Limit the max power
    if (Math.abs(drivePower) > 0.5) {
      drivePower = Math.copySign(0.5, drivePower);
    }

    // If we sense a large change in error, apply a small negative power to counteract the board tipping
    if (changeInAngle > 5) {
      drivePower = Math.copySign(0.2, -drivePower); // Reverse the motors from what we previously told them to do
    }

    m_DriveSubsystem.drive(drivePower, drivePower);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(error) < 1;
  }
}