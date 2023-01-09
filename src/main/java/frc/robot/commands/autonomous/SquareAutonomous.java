package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

public class SquareAutonomous extends SequentialCommandGroup{
    public SquareAutonomous(){
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 1));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 1));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new DriveForTimeCommand(0.2, 0.2, 2));
    addCommands(new GyroTurnToAngleCommand(90));
    }
}
