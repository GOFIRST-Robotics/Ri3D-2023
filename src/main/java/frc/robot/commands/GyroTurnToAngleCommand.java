// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class GyroTurnToAngleCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem; // drive system
  double targetAngle; // the target angle we wish to achieve
  double kp; // scaling factor based off Constants.GYRO_KP
  double error; // How "incorrect" the current angle of the robot is as its moving

  /** Creates a new GyroTurnToAngle. */
  public GyroTurnToAngleCommand(double targetAngle, boolean relativeToCurrent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = Robot.m_driveSubsystem;
    this.targetAngle = targetAngle + (relativeToCurrent ? m_DriveSubsystem.getAngle() : 0); // conditonal operator, either gives relative angle or zero
    kp = Constants.GYRO_KP; // scaling factor for angle adjustment in execute() (line 34)
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - m_DriveSubsystem.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which "gets" our current angle from the robot
    double value = Math.min(error*kp, 1); // Multiply by scaling factor kp to determine motor percent power between 1 and 100 percent

    m_DriveSubsystem.drive(-value, value); // write calculated values to m_DriveSubsystem
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1;
  }
}