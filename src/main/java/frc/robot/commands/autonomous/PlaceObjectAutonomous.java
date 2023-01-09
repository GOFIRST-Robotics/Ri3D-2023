package frc.robot.commands.autonomous;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.Robot;
import frc.robot.commands.DriveForTimeCommand;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.GyroTurnToAngle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (2) ******************************************************
 * This is a custom autonomous routine */
public class PlaceObjectAutonomous extends SequentialCommandGroup {
  private ExtenderSubsystem m_extenderSubsystem = Robot.m_extenderSubsystem;
  private GrabberSubsystem m_grabberSubsystem = Robot.m_grabberSubsystem;
  // List commands here sequentially
  public PlaceObjectAutonomous() {
    addCommands(new DriveToAprilTagCommand()); //find and drive to aprilTags

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4))); //fully extend arm

    addCommands(new InstantCommand(() -> m_grabberSubsystem.extend())); //drop object by extending grabber holding it, keep grabber open to make tele-op easier

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0))); // fully retract arm
  }
}