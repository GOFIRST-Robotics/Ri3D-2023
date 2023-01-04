// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private VictorSP m_leftFeeder;
  private VictorSP m_rightFeeder;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    m_leftFeeder = new VictorSP(Constants.FEEDER_LEFT_MOTOR_ID);
    m_rightFeeder = new VictorSP(Constants.FEEDER_RIGHT_MOTOR_ID);

    m_leftFeeder.setInverted(Constants.FEEDER_INVERT_LEFT);
    m_rightFeeder.setInverted(Constants.FEEDER_INVERT_RIGHT);
  }

  public void setMotors(double power) {
    m_leftFeeder.set(power);
    m_rightFeeder.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
