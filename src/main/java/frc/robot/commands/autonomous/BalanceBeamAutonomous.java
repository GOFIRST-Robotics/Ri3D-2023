package frc.robot.commands.autonomous;

import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveForTimeCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (1) ******************************************************
 * This is a custom autonomous routine */
public class BalanceBeamAutonomous extends SequentialCommandGroup {

  // List commands here sequentially
  public BalanceBeamAutonomous () {
    addCommands(new DriveForTimeCommand(.2, .2, 1));
    addCommands(new BalanceOnBeamCommand());
  }
}