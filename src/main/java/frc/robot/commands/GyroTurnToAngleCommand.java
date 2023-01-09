// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class GyroTurnToAngleCommand extends CommandBase {

  DriveSubsystem m_DriveSubsystem; // drive system
  double degreesToTurn; // the target angle we wish to achieve
  double error; // How "incorrect" the current angle of the robot is as its moving
  double targetAngle;

  /** Creates a new GyroTurnToAngle. */
  public GyroTurnToAngleCommand(double degreesToTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_DriveSubsystem);
    this.degreesToTurn = degreesToTurn;
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
    error = targetAngle - m_DriveSubsystem.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which "gets" our current angle from the robot
    double value = error*Constants.GYRO_KP; // Multiply by scaling factor kp to determine motor percent power between 1 and 100 percent
    if (Math.abs(value) > 0.75) {
      value = Math.copySign(0.75, value);
    }
    if (Math.abs(value) < 0.15) {
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
    return Math.abs(error) < 1;
  }
}