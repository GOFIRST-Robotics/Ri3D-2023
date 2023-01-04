// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX m_extenderMasterMotor;
  private TalonSRX m_extenderFollowerMotor;
  private TalonSRX m_motor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_extenderMasterMotor = new TalonSRX(Constants.INTAKE_EXTENDER_MASTER_MOTOR_ID);
    m_extenderFollowerMotor = new TalonSRX(Constants.INTAKE_EXTENDER_FOLLOWER_MOTOR_ID);
    m_motor = new TalonSRX(Constants.INTAKE_MOTOR_ID);

    m_extenderMasterMotor.configFactoryDefault();
    m_extenderFollowerMotor.configFactoryDefault();
    m_motor.configFactoryDefault();

    m_extenderFollowerMotor.follow(m_extenderMasterMotor);

    m_extenderMasterMotor.setInverted(Constants.INTAKE_EXTENDER_MASTER_INVERT);
    m_extenderFollowerMotor.setInverted(Constants.INTAKE_EXTENDER_FOLLOWER_INVERT);
    m_motor.setInverted(Constants.INTAKE_INVERT);

    m_extenderMasterMotor.setNeutralMode(NeutralMode.Brake);
    m_extenderFollowerMotor.setNeutralMode(NeutralMode.Brake);
    m_motor.setNeutralMode(Constants.INTAKE_NEUTRAL);
  }

  public void setMotor(double value) {
    m_motor.set(ControlMode.PercentOutput, value);
  }

  public void setExtender(double value) {
    m_extenderMasterMotor.set(ControlMode.PercentOutput, value);
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