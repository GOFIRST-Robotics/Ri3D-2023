// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class GyroTurnToAngle extends CommandBase {

  DriveSubsystem m_DriveSubsystem;
  double targetAngle;
  double kp;
  double error;

  /** Creates a new GyroTurnToAngle. */
  public GyroTurnToAngle(double targetAngle, boolean relativeToCurrent) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = Robot.m_driveSubsystem;
    this.targetAngle = targetAngle + (relativeToCurrent ? m_DriveSubsystem.getAngle() : 0);
    kp = Constants.GYRO_KP;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - m_DriveSubsystem.getAngle();
    double value = Math.min(error*kp, 1);

    m_DriveSubsystem.drive(-value, value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return error < 1;
  }
}
