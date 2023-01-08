package frc.robot.commands.autonomous;

import frc.robot.commands.DriveForTime;
import frc.robot.commands.GyroTurnToAngle;
import frc.robot.commands.autonomous.helperCommands.Wait;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (2) ******************************************************
 * This is a custom autonomous routine */
public class AutonomousMode_2 extends SequentialCommandGroup {

  public AutonomousMode_2() {
    addCommands(new DriveForTime(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTime(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTime(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
    addCommands(new DriveForTime(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngle(90, true));
  }
}