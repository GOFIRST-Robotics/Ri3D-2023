// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** TimedGyroDriveStraightCommand **************************************************
 * Drives straoight using gyroscope feedback for a specified number of seconds. */
public class TimedGyroDriveStraightCommand extends CommandBase {

  private DriveSubsystem m_drivetrainSubsystem;

	double duration; // how long we want to move forward
	Timer timer = new Timer();
    double startAngle;
    double driveRate;

	public TimedGyroDriveStraightCommand(double time, double driveRate) {
	  duration = time;
      this.driveRate = driveRate;
      m_drivetrainSubsystem = Robot.m_driveSubsystem;
      addRequirements(m_drivetrainSubsystem);
	}

	//Starts the timer and gets the starting angle
	public void initialize() {
	  timer.reset();
  	  timer.start();
      startAngle = m_drivetrainSubsystem.getAngle();
	}

	// Called every time the scheduler runs while the command is scheduled.
    public void execute() {
      double error = startAngle - m_drivetrainSubsystem.getAngle(); // Correction needed for robot angle (our starting angle, since we would like to drive straight)
      double value1 = Math.min(driveRate + Constants.GYRO_TURN_KP * error, 1); // plus or minus Constants.GYRO_KP * error, meant for error correction
      double value2 = Math.min(driveRate - Constants.GYRO_TURN_KP * error, 1);
      m_drivetrainSubsystem.drive(value1, value2); // write percent values to motors
	}
	
	// returns true when the command ends
	public boolean isFinished() {
		return timer.get() >= duration;
	}

	// Called once the command ends or is interrupted.	
	public void end(boolean interrupted) {
		timer.reset();
	}
}