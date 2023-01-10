// Author: UMN Robotics Ri3d
// Last Updated : January 2023

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
  SendableChooser<Boolean> driveChooser = new SendableChooser<Boolean>(); // Create a chooser to select between tank drive and arcade drive


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
    if (driveChooser.getSelected()) { // Tank Drive
      Double left_power = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * m_subsystem.CURRENT_DRIVE_SCALE; // -1 inverts the controller axis
      Double right_power = -1 * Robot.controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS) * m_subsystem.CURRENT_DRIVE_SCALE; // -1 inverts the controller axis
      m_subsystem.drive(left_power, right_power);
    } else { // Arcade Drive
      Double turning_power = -1 * Robot.controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
      Double drive_power = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
      m_subsystem.drive(drive_power - turning_power, drive_power + turning_power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}