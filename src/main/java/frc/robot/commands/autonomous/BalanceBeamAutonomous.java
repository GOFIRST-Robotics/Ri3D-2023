// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.TimedGyroDriveStraightCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Self-Balancing) ******************************************************
 * This is an autonomous routine for driving onto the charging station and then self-balancing using the gyroscope pitch as feedback. */
public class BalanceBeamAutonomous extends SequentialCommandGroup {
  
  public BalanceBeamAutonomous () { // List commands here sequentially
    addCommands(new TimedGyroDriveStraightCommand(1, .2));

    addCommands(new BalanceOnBeamCommand());
  }
}