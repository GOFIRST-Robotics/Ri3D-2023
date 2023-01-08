// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private VictorSP m_motor;
  private DigitalInput limitSwitch = new DigitalInput(0);

  /** Creates a new Extender ubsystem. */
  public ExtenderSubsystem() {
    m_motor = new VictorSP(Constants.EXTENDER_MOTOR_ID);

    m_motor.setInverted(Constants.EXTENDER_INVERT);
  }

  public void setMotor(double power) {
    m_motor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}