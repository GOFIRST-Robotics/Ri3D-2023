package frc.robot.commands.autonomous;

import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.GyroTurnToAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (2) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_2 extends SequentialCommandGroup {

  // List commands here sequentially
  public AutonomousMode_2() {
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
  }
}