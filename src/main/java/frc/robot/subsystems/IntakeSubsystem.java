// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private VictorSP m_extenderMasterMotor;
  private VictorSP m_motor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_extenderMasterMotor = new VictorSP(Constants.INTAKE_EXTENDER_MASTER_MOTOR_ID);
    m_motor = new VictorSP(Constants.INTAKE_MOTOR_ID);

    m_extenderMasterMotor.setInverted(Constants.INTAKE_EXTENDER_MASTER_INVERT);
    m_motor.setInverted(Constants.INTAKE_INVERT);
  }

  public void setMotor(double value) {
    m_motor.set(value);
  }

  public void setExtender(double value) {
    m_extenderMasterMotor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}