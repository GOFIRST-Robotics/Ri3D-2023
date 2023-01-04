package frc.robot.commands.autonomous;

import frc.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (1) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_1 extends SequentialCommandGroup {

  public AutonomousMode_1() {
    addCommands(new Wait(2)); // List commands here sequentially
  }
}