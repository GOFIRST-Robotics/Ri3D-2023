package frc.robot.commands.autonomous;

import frc.robot.commands.DriveForTime;
import frc.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (1) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_1 extends SequentialCommandGroup {

  public AutonomousMode_1() {
    addCommands(new DriveForTime(0.2, 0.2, 5));
    addCommands(new Wait(2)); // List commands here sequentially
    addCommands(new DriveForTime(-0.2, -0.2, 5));
  }
}