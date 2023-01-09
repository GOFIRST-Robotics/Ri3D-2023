// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

// This command takes the joystick inputs and demands that the drivetrain follow them
public class DriveCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  SendableChooser<Boolean> driveChooser = new SendableChooser<Boolean>();

  /** Default drive command that takes the joystick inputs and demands that the drivetrain follow them */
  public DriveCommand() {
    m_subsystem = Robot.m_driveSubsystem;
    addRequirements(m_subsystem);

    // Drive Modes //
    driveChooser.setDefaultOption("Tank Drive", true);
    driveChooser.addOption("Arcade Drive", false);

    SmartDashboard.putData("Drive Mode", driveChooser);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveChooser.getSelected()) {
      Double left_power = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
      Double right_power = -1 * Robot.controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS);
      m_subsystem.drive(left_power, right_power);
    } else {
      Double turning_power = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
      Double drive_power = -1 * Robot.controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS);
      m_subsystem.drive(drive_power + turning_power, drive_power - turning_power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish
  }
}