// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private TalonSRX m_motor1;
  private TalonSRX m_motor2;

  public ClimberSubsystem() {
    m_motor1 = new TalonSRX(Constants.CLIMBER_1_MOTOR_ID);
    m_motor2 = new TalonSRX(Constants.CLIMBER_2_MOTOR_ID);

    m_motor1.configFactoryDefault();
    m_motor2.configFactoryDefault();

    m_motor1.setInverted(Constants.CLIMBER_1_INVERT);
    m_motor2.setInverted(Constants.CLIMBER_2_INVERT);

    m_motor1.setNeutralMode(Constants.CLIMBER_1_NEUTRAL);
    m_motor2.setNeutralMode(Constants.CLIMBER_2_NEUTRAL);
  }

  public void setMotors(double value) {
    m_motor1.set(ControlMode.PercentOutput, value);
    m_motor2.set(ControlMode.PercentOutput, value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}