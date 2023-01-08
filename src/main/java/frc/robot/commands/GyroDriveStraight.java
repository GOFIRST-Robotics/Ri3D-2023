package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** Wait *******************************************************
 * Waits for a specified number of seconds. Useful for autonomous command groups! */
public class GyroDriveStraight extends CommandBase {

  private DriveSubsystem m_drivetrainSubsystem;

	double duration;
	Timer timer = new Timer();
  double startAngle;
  double driveRate;

	public GyroDriveStraight(double time, double driveRate) {
		duration = time;
    this.driveRate = driveRate;
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    addRequirements(m_drivetrainSubsystem);
	}

	public void initialize() {
		timer.reset();
		timer.start();
    startAngle = m_drivetrainSubsystem.getAngle();
	}

  public void execute() {
    double error = startAngle - m_drivetrainSubsystem.getAngle();
    double value1 = Math.min(driveRate + Constants.GYRO_KP * error, 1);
    double value2 = Math.min(driveRate - Constants.GYRO_KP * error, 1);
    // System.out.println("Error " + error);
    // System.out.println("Value: " + value1);
    // System.out.println("Value: " + value2);
    // System.out.println(m_drivetrainSubsystem.getAngle());
    m_drivetrainSubsystem.drive(value1, value2);
  }
	
	public boolean isFinished() {
		return timer.get() >= duration;
	}

	public void end(boolean interrupted) {
		timer.reset();
    System.out.println("ENDED");
	}
}