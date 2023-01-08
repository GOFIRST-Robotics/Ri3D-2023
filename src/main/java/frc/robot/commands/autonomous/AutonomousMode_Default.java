package frc.robot.commands.autonomous;

import frc.robot.commands.GyroDriveStraightCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine */
public class AutonomousMode_Default extends SequentialCommandGroup {

  // List commands here sequentially
  public AutonomousMode_Default() {
    addCommands(new GyroDriveStraightCommand(5, 0.3)); // List commands here sequentially
  }
}