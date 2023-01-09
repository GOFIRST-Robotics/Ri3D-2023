package frc.robot.commands.autonomous;

import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Score Pre-Loaded Cube) ******************************************************
 * This is an autonomous routine  for scoring a pre-loaded cube by autonomously driving to the nearest Apriltag */
public class PlaceCubeAutonomous extends SequentialCommandGroup {
  private ExtenderSubsystem m_extenderSubsystem = Robot.m_extenderSubsystem;
  private GrabberSubsystem m_grabberSubsystem = Robot.m_grabberSubsystem;

  public PlaceCubeAutonomous() { // List commands here sequentially
    addCommands(new DriveToAprilTagCommand(0.5)); //find and drive to aprilTags

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4))); //fully extend arm

    addCommands(new InstantCommand(() -> m_grabberSubsystem.extend())); //drop object by extending grabber holding it, keep grabber open to make tele-op easier

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0))); // fully retract arm
  }
}