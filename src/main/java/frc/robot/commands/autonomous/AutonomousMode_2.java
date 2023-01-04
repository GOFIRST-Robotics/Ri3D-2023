package frc.robot.commands.autonomous;

import frc.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (2) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_2 extends SequentialCommandGroup {

  public AutonomousMode_2() {
    addCommands(new Wait(2)); // List commands here sequentially
  }
}