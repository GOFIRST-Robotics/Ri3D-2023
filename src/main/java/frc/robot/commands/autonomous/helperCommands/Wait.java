// Author: Anthony Brogni (brogn002@umn.edu)
// Last Updated : 2022

package frc.robot.commands.autonomous.helperCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Wait *******************************************************
 * Waits for a specified number of seconds. Useful for autonomous command groups! */
public class Wait extends CommandBase {

	double duration;
	Timer timer = new Timer();

	public Wait(double time) {
		duration = time;
	}

	public void initialize() {
		timer.reset();
		timer.start();
	}
	
	public boolean isFinished() {
		return timer.get() >= duration;
	}

	public void end(boolean interrupted) {
		timer.reset();
	}
}