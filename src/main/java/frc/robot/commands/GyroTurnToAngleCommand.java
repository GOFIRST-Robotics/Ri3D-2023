// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

//  uses PID to align to angle using gyro
public class GyroTurnToAngleCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem; // drive system
  double degreesToTurn; // the target angle we wish to achieve
  double error; // How "incorrect" the current angle of the robot is as its moving
  double targetAngle;
  boolean loopGyro;

  /** Turns to an angle relative to the current angle using the gyro */
  public GyroTurnToAngleCommand(double degreesToTurn) {
    m_DriveSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
    this.degreesToTurn = degreesToTurn;
  }

  public GyroTurnToAngleCommand(double degreesToTurn, boolean nonRelativeToCurrentRotation) {
    this(degreesToTurn);
    loopGyro = nonRelativeToCurrentRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetAngle = degreesToTurn + m_DriveSubsystem.getAngle();
    System.out.println("CURRENT ANGLE" + m_DriveSubsystem.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (loopGyro) {
      error = targetAngle - m_DriveSubsystem.getAngle() % 360;
    } else {
      error = targetAngle - m_DriveSubsystem.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which "gets" our current angle from the robot
    }
    double value = error * Constants.GYRO_TURN_KP; // Multiply by scaling factor kp to determine motor percent power between 1 and 100 percent
    if (Math.abs(value) > 0.75) {
      value = Math.copySign(0.75, value);
    }
    if (Math.abs(value) < 0.15) { // Min drive value
      value = Math.copySign(0.15, value);
    }
    System.out.println("error" + error);
    System.out.println("target" + targetAngle);
    System.out.println("degrees to turn" + degreesToTurn);
    System.out.println("value" + value);
    SmartDashboard.putNumber("Error", error);
    m_DriveSubsystem.drive(value, -value); // write calculated values to m_DriveSubsystem
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("ENDED");
      m_DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.DRIVE_TURNING_THRESHOLD_DEGREES;
  }
}