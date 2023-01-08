// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {
  private TalonSRX m_motor_1; // This will be the one with the encoder connected
  private TalonSRX m_motor_2;
  private Solenoid lower_the_extender;
  private boolean isRaised;
  private double positionZero;
  public int currentSetpoint;

  /** Creates a new Extender ubsystem. */
  public ExtenderSubsystem() {
    m_motor_1 = new TalonSRX(Constants.EXTENDER_MOTOR_1_ID); // This will be the one with the encoder connected
    m_motor_2 = new TalonSRX(Constants.EXTENDER_MOTOR_2_ID);

    m_motor_1.configFactoryDefault();
    m_motor_2.configFactoryDefault();

    m_motor_1.setNeutralMode(NeutralMode.Brake);
    m_motor_2.setNeutralMode(NeutralMode.Brake);

    m_motor_1.setInverted(Constants.EXTENDER_INVERT);
    m_motor_2.setInverted(Constants.EXTENDER_INVERT);

    m_motor_1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.EXTENDER_PIDIDX, Constants.EXTENDER_TIMEOUT);
		m_motor_1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.EXTENDER_ENCODER_FRAME_RATE, Constants.EXTENDER_TIMEOUT);
		m_motor_1.setSensorPhase(Constants.EXTENDER_SENSOR_PHASE); // Ensures that the encoder is "in phase" with the Talon

    m_motor_2.follow(m_motor_1); // motor 2 will follow motor 1

    lower_the_extender = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LOWER_THE_EXTENDER_ID);
    isRaised = true;

    currentSetpoint = 0;
  }

  // Motor Methods //

  public void setPower(double power) {
    m_motor_1.set(ControlMode.PercentOutput, power); // motor 2 will follow motor 1
  }

  public void stop() {
    m_motor_1.set(ControlMode.PercentOutput, 0);
    m_motor_2.set(ControlMode.PercentOutput, 0);
  }

  // Raise/Lower Piston Methods //

  public void lowerExtender() {
    lower_the_extender.set(false);
    isRaised = false;
  }

  public void raiseExtender() {
    lower_the_extender.set(true);
    isRaised = true;
  }

  public void toggleExtenderRaiser() {
    if (isRaised) {
      lowerExtender();
    } else {
      raiseExtender();
    }
  }

  // Encoder Methods //

  /** Reset the encoder zero position ***************************************/
	public void resetEncoder() {
		positionZero = m_motor_1.getSelectedSensorPosition(Constants.EXTENDER_PIDIDX);
	}

	/** Get the encoder position and velocity *********************************
	 * Encoder position returns counts
	 * Encoder speed returns counts per 100 ms, convert to RPM for output */
	public double getEncoderPosition() {
		return (m_motor_1.getSelectedSensorPosition(Constants.EXTENDER_PIDIDX) - positionZero);
	}
	public double getEncoderSpeed() {
		int cp100ms = (int)(m_motor_1.getSelectedSensorVelocity(Constants.EXTENDER_PIDIDX));
		
		return (double)cp100ms * 10.0 * 60.0 / Constants.EXTENDER_ENCODER_COUNTS_PER_REV;
	}
	public double getEncoderPositionZero() {
		return positionZero;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}