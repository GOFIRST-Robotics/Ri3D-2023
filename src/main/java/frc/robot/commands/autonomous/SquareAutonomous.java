package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveStraightCommand;
import frc.robot.commands.GyroTurnToAngleCommand;

public class SquareAutonomous extends SequentialCommandGroup{
    public SquareAutonomous(){
    addCommands(new GyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new GyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new GyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    addCommands(new GyroDriveStraightCommand(2, 0.2));
    addCommands(new GyroTurnToAngleCommand(90));
    }
}
