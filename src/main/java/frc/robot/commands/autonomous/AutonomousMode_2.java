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
public class AutonomousMode_2 extends SequentialCommandGroup {
  private ExtenderSubsystem m_extenderSubsystem = Robot.m_extenderSubsystem;
  private GrabberSubsystem m_grabberSubsystem = Robot.m_grabberSubsystem;
  // List commands here sequentially
  public AutonomousMode_2() {
    addCommands(new DriveToAprilTagCommand());

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4)));

    addCommands(new InstantCommand(() -> m_grabberSubsystem.extend()));

    addCommands(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0)));



   
  }
}