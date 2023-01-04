// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private TalonSRX m_leftFrontMotor;
  private TalonSRX m_rightFrontMotor;
  private TalonSRX m_leftRearMotor;
  private TalonSRX m_rightRearMotor;

  public DriveSubsystem() {
    m_leftFrontMotor = new TalonSRX(Constants.LEFT_FRONT_DRIVE_MOTOR_ID);
    m_rightFrontMotor = new TalonSRX(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID);
    m_leftRearMotor = new TalonSRX(Constants.LEFT_REAR_DRIVE_MOTOR_ID);
    m_rightRearMotor = new TalonSRX(Constants.RIGHT_REAR_DRIVE_MOTOR_ID);

    m_leftFrontMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightFrontMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
    m_leftRearMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightRearMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
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
}