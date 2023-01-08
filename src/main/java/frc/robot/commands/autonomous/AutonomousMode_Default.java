package frc.robot.commands.autonomous;

import frc.robot.commands.GyroDriveStraight;
import frc.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine */
public class AutonomousMode_Default extends SequentialCommandGroup {

  public AutonomousMode_Default() {
    addCommands(new GyroDriveStraight(5, 0.3)); // List commands here sequentially
  }
}