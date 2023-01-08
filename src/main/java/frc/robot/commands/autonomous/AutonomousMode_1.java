package frc.robot.commands.autonomous;

import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.autonomous.helperCommands.Wait;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (1) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_1 extends SequentialCommandGroup {

  // List commands here sequentially
  public AutonomousMode_1() {
    addCommands(new DriveForTimeCommand(.2, .2, 1));
    addCommands(new BalanceOnBeamCommand());
  
    /*   addCommands(new DriveForTimeCommand(0.2, 0.2, 5));
    addCommands(new Wait(2)); 
    addCommands(new DriveForTimeCommand(-0.2, -0.2, 5));*/
  }
}