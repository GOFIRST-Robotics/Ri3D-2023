// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private DoubleSolenoid m_solenoid;
  private TalonSRX m_motor;

  private boolean m_shooterOn = false;

  public ShooterSubsystem() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SHOOTER_SOLENOID_1, Constants.SHOOTER_SOLENOID_2);
    m_motor = new TalonSRX(Constants.SHOOTER_MOTOR_ID);

    m_motor.configFactoryDefault();
    m_motor.setInverted(Constants.SHOOTER_INVERT);
    m_motor.setNeutralMode(NeutralMode.Coast);

    m_solenoid.set(Constants.SHOOTER_DEFAULT_POSITION);
  }

  public void toggleSolenoid() {
    m_solenoid.toggle();
  }

  public void toggleShooter() {
    m_shooterOn = !m_shooterOn;
    if (m_shooterOn)
      m_motor.set(ControlMode.PercentOutput, Constants.SHOOTER_SPEED);
    else
      m_motor.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopShooter() {
    m_shooterOn = false;
    m_motor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
