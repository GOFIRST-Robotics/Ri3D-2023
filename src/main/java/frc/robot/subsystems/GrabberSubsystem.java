// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  private VictorSP m_motor;

  /** Creates a new Grabber Subsystem. */
  public GrabberSubsystem() {
    m_motor = new VictorSP(Constants.GRABBERMOTOR_ID);

    m_motor.setInverted(Constants.GRABBER_INVERT);
  }

  public void setMotor(double value) {
    m_motor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set the default command for the subsystem (null by default)
  public void initDefaultCommand() {
		setDefaultCommand(null);
	}
}