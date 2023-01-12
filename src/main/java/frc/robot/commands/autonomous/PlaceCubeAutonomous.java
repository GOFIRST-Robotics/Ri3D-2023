// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.commands.DriveToTrackedTargetCommand;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Score Pre-Loaded Cube) ******************************************************
 * This is an autonomous routine  for scoring a pre-loaded cube by autonomously driving to the nearest Apriltag */
public class PlaceCubeAutonomous extends SequentialCommandGroup {
  private ExtenderSubsystem m_extenderSubsystem = Robot.m_extenderSubsystem;
  private GrabberSubsystem m_grabberSubsystem = Robot.m_grabberSubsystem;

  public PlaceCubeAutonomous() { // List commands here sequentially
    addCommands(new DriveToTrackedTargetCommand(0.5)); // find and drive to nearest aprilTag

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4))); // fully extend

    addCommands(new InstantCommand(() -> m_grabberSubsystem.extend())); // drop the cube  by extending grabber holding it

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0))); // fully retract
  }
}