// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
  
  private TalonSRX m_leftFrontMotor;
  private TalonSRX m_rightFrontMotor;
  private TalonSRX m_leftRearMotor;
  private TalonSRX m_rightRearMotor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftFrontMotor = new TalonSRX(Constants.LEFT_FRONT_DRIVE_MOTOR_ID);
    m_rightFrontMotor = new TalonSRX(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID);
    m_leftRearMotor = new TalonSRX(Constants.LEFT_REAR_DRIVE_MOTOR_ID);
    m_rightRearMotor = new TalonSRX(Constants.RIGHT_REAR_DRIVE_MOTOR_ID);

    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftRearMotor.configFactoryDefault();
    m_rightRearMotor.configFactoryDefault();

    m_leftFrontMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightFrontMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
    m_leftRearMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightRearMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);

    m_leftFrontMotor.setNeutralMode(Constants.DRIVE_NEUTRAL);
    m_rightFrontMotor.setNeutralMode(Constants.DRIVE_NEUTRAL);
    m_leftRearMotor.setNeutralMode(Constants.DRIVE_NEUTRAL);
    m_rightRearMotor.setNeutralMode(Constants.DRIVE_NEUTRAL);
  }

  public void drive(double left, double right) {
    m_leftFrontMotor.set(ControlMode.PercentOutput, left);
    m_rightFrontMotor.set(ControlMode.PercentOutput, right);
    m_leftRearMotor.set(ControlMode.PercentOutput, left);
    m_rightRearMotor.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set the default command for the subsystem
  public void initDefaultCommand() {
		setDefaultCommand(new DriveCommand()); // By default we want teleop driving
	}
}