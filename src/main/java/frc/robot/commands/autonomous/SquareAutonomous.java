// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

// This autonomous routine drives in a square!
public class SquareAutonomous extends SequentialCommandGroup{
    public SquareAutonomous(){
    addCommands(new DriveForDistanceCommand(0.5, 0.2)); // Drive straight
    addCommands(new GyroTurnToAngleCommand(90)); // Turn 90 degrees
    addCommands(new DriveForDistanceCommand(0.5, 0.2)); // Drive straight
    addCommands(new GyroTurnToAngleCommand(90)); // Turn 90 degrees
    addCommands(new DriveForDistanceCommand(0.5, 0.2)); // Drive straight
    addCommands(new GyroTurnToAngleCommand(90)); // Turn 90 degrees
    addCommands(new DriveForDistanceCommand(0.5, 0.2)); // Drive straight
    addCommands(new GyroTurnToAngleCommand(90)); // Turn 90 degrees
    }
}