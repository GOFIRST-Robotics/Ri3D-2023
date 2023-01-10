// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// This command is used to turn the robot to face an AprilTag, using the VisionSubsystem to detect the AprilTag
public class TurnToAprilTagCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem; // drive system
  VisionSubsystem m_VisionSubsystem; // vision system
  double targetAngle; // Future variable, probably used in the case where the camera is not placed horizionally in front of the robot
  double kp; // scaling ratio for robot movement
  double error; // amount of error our robot detects that it tries to correct for, relative to the position of the AprilTag

  /** Turns the robot using the gyro only for following an April Tag (tracked target from VisionSubsystem) */
  public TurnToAprilTagCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = Robot.m_driveSubsystem; // define drive and vision system
    m_VisionSubsystem = Robot.m_visionSubsystem;
    kp = Constants.TRACKED_TAG_ROATION_KP; // Calculate kp scaling ratio based off scaled GYRO_KP value (used in execute())
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_VisionSubsystem.getHasTarget()) { // if we find a target,
      error = m_VisionSubsystem.getBestTarget().getYaw(); // calculate error based off Yaw value of our current best target
      double value = -Math.min(error*kp, 1); // calculate motor percentage value

      m_DriveSubsystem.drive(-value, value); // write values to motors, negative and positive value in order for turning to occur
    } else {
      m_DriveSubsystem.stop(); // otherwise, don't do anything
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(error) < 1;
  }
}