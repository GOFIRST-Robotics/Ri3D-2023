// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private VictorSP m_motor1;
  private VictorSP m_motor2;

  public ClimberSubsystem() {
    m_motor1 = new VictorSP(Constants.CLIMBER_1_MOTOR_ID);
    m_motor2 = new VictorSP(Constants.CLIMBER_2_MOTOR_ID);

    m_motor1.setInverted(Constants.CLIMBER_1_INVERT);
    m_motor2.setInverted(Constants.CLIMBER_2_INVERT);
  }

  public void setMotors(double value) {
    m_motor1.set(value);
    m_motor2.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
