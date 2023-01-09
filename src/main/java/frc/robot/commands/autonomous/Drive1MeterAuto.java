package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine. It drives forward for 2 seconds, turns around 180 degrees, and then drives back. */
public class Drive1MeterAuto extends SequentialCommandGroup {

  // List commands here sequentially
  public Drive1MeterAuto() { // List commands here sequentially
    addCommands(new DriveForDistanceCommand(1, 0.3));
  }
}