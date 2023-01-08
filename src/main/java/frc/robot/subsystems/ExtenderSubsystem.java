// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private VictorSP m_motor_1;
  private VictorSP m_motor_2;

  /** Creates a new Extender ubsystem. */
  public ExtenderSubsystem() {
    m_motor_1 = new VictorSP(Constants.EXTENDER_MOTOR_1_ID);
    m_motor_2 = new VictorSP(Constants.EXTENDER_MOTOR_2_ID);

    m_motor_1.setInverted(Constants.EXTENDER_INVERT);
    m_motor_2.setInverted(Constants.EXTENDER_INVERT);
  }

  public void setPower(double power) {
    m_motor_1.set(power);
    m_motor_2.set(power);
  }

  public void stop() {
    m_motor_1.set(0);
    m_motor_2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}