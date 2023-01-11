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
	private Timer timer = new Timer();

	double duration; // how long we want to drive
    double startAngle; // our starting gyroscope heading
    double driveRate; // how fast we want to drive

	public TimedGyroDriveStraightCommand(double time, double driveRate) {
	  this.duration = time;
      this.driveRate = driveRate;
      this.m_drivetrainSubsystem = Robot.m_driveSubsystem;
      addRequirements(m_drivetrainSubsystem);
	}

	//Starts the timer and gets the starting angle
	public void initialize() {
	  timer.reset();
  	  timer.start();
      this.startAngle = m_drivetrainSubsystem.getAngle(); // Get our initial gyroscope heading; this will be the 'goal' of our PID loop
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