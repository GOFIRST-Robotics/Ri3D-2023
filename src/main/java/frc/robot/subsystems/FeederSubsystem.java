// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private TalonSRX m_leftFeeder;
  private TalonSRX m_rightFeeder;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    m_leftFeeder = new TalonSRX(Constants.FEEDER_LEFT_MOTOR_ID);
    m_rightFeeder = new TalonSRX(Constants.FEEDER_RIGHT_MOTOR_ID);

    m_leftFeeder.configFactoryDefault();
    m_rightFeeder.configFactoryDefault();

    m_leftFeeder.setInverted(Constants.FEEDER_INVERT_LEFT);
    m_rightFeeder.setInverted(Constants.FEEDER_INVERT_RIGHT);

    m_leftFeeder.setNeutralMode(Constants.FEEDER_NEUTRAL_LEFT);
    m_rightFeeder.setNeutralMode(Constants.FEEDER_NEUTRAL_RIGHT);
  }

  public void setMotors(double power) {
    m_leftFeeder.set(ControlMode.PercentOutput, power);
    m_rightFeeder.set(ControlMode.PercentOutput, power);
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