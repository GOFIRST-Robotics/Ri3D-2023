// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine. It drives forward for 2 seconds, turns around 180 degrees, and then drives back. */
public class AutonomousMode_Default extends SequentialCommandGroup {

  // List commands here sequentially
  public AutonomousMode_Default() { // List commands here sequentially
    addCommands(new TimedGyroDriveStraightCommand(2, 0.3));

    addCommands( new GyroTurnToAngleCommand(180));

    addCommands(new TimedGyroDriveStraightCommand(2, 0.3));
  }
}