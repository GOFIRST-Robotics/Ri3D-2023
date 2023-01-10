// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.DriveInFrontOfTag;
import frc.robot.commands.ExtenderControlCommand;
import frc.robot.commands.GrabCommand;
import frc.robot.commands.GyroTurnToAngleCommand;
import frc.robot.commands.ReleaseCommand;

public class AdvancedAuto extends SequentialCommandGroup {
  /** Creates a new AdvancedAuto. */
  public AdvancedAuto() {
    addCommands(new DriveInFrontOfTag(0.5)); // Drive to tag
    addCommands(new ExtenderControlCommand()); // Bring arm up
    addCommands(new ReleaseCommand()); // Release cube
    addCommands(new DriveForDistanceCommand(-0.4, 0.7)); // Backup to turn
    addCommands(new ExtenderControlCommand()); // Lower extender so low COG
    addCommands(new GyroTurnToAngleCommand(90)); //Figure out if positive or negative, turns 90
    addCommands(new DriveForDistanceCommand(0.5, 0.8)); // Drives to side
    addCommands(new GyroTurnToAngleCommand(90)); //Figure out if positive or negative, turns 90
    addCommands(new DriveForDistanceCommand(2, 0.8)); // Drives foward towards nearest cube 
    addCommands(new GrabCommand());
    addCommands(new DriveForDistanceCommand(-2, 0.8)); // Go back
    addCommands(new GyroTurnToAngleCommand(-90));
    addCommands(new DriveForDistanceCommand(0.2, 0.5));
    addCommands(new GyroTurnToAngleCommand(-90));
    addCommands(new DriveInFrontOfTag(0.5));
    addCommands(new ExtenderControlCommand());
    addCommands(new ReleaseCommand());
    addCommands(new DriveForDistanceCommand(-0.4, 0.7)); // Backup to turn
    addCommands(new ExtenderControlCommand()); // Lower extender so low COG
    addCommands(new GyroTurnToAngleCommand(180)); //Figure out if positive or negative, turns 90
    addCommands(new DriveForDistanceCommand(1, 0.5)); // Drives to side
    addCommands(new BalanceBeamAutonomous());
  }
}
