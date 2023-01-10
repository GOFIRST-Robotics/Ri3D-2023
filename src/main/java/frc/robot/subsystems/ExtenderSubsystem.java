// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {
  private TalonSRX m_motor_1; // This will be the Talon SRX with the encoder connected
  private TalonSRX m_motor_2;
  private DoubleSolenoid lower_the_extender; // Our extender has a pneumatic piston for raising/lowering the whole subsystem
  private boolean isRaised; // Keeps track of whether the extender is currently raised (piston extended)
  private double positionZero;
  public int currentSetpoint;

  /** Subsystem for controlling the extender */
  public ExtenderSubsystem() {
    m_motor_1 = new TalonSRX(Constants.EXTENDER_MOTOR_1_ID); // This will be the Talon SRX with the encoder connected
    m_motor_2 = new TalonSRX(Constants.EXTENDER_MOTOR_2_ID);

    // Configure our Talon SRX motor controllers
    m_motor_1.configFactoryDefault();
    m_motor_2.configFactoryDefault();
    m_motor_1.setNeutralMode(NeutralMode.Brake);
    m_motor_2.setNeutralMode(NeutralMode.Brake);
    m_motor_1.setInverted(Constants.EXTENDER_INVERT);
    m_motor_2.setInverted(Constants.EXTENDER_INVERT);

    // Config our quadrature encoder
    m_motor_1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.EXTENDER_PIDIDX, Constants.EXTENDER_TIMEOUT);
		m_motor_1.setSensorPhase(Constants.EXTENDER_SENSOR_PHASE); // Ensures that the encoder is "in phase" with the Talon

    m_motor_2.follow(m_motor_1); // motor 2 will follow motor 1

    // Instantiate the solenoid
    lower_the_extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.LOWER_THE_EXTENDER_ID_1, Constants.LOWER_THE_EXTENDER_ID_2);
    isRaised = true; // The extender will start raised

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

  // Methods for controlling the state of the solenoid //
  public void lowerExtender() {
    lower_the_extender.set(Value.kForward);
    isRaised = false;
  }
  public void raiseExtender() {
    lower_the_extender.set(Value.kReverse);
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
	public void resetEncoder() { // Reset the 'zero' position to be the current encoder position
		positionZero = m_motor_1.getSelectedSensorPosition(Constants.EXTENDER_PIDIDX);
	}
	public double getEncoderPosition() { // Returns the current encoder position in raw encoder counts
		return (m_motor_1.getSelectedSensorPosition(Constants.EXTENDER_PIDIDX) - positionZero);
	}
	public double getEncoderPositionZero() { // Returns the current 'zero' position in raw encoder counts
		return positionZero;
	}

  // Methods for changing the setpoint/goal of this subsystem's default command //
  public int getCurrentSetPoint() {
    return currentSetpoint;
  }
  public void changeSetpoint(int newSetPoint) {
    if (newSetPoint <= 4 && newSetPoint >= 0) {
      currentSetpoint = newSetPoint;
    }
  }
  public void incrementSetPoint() {
    currentSetpoint = Math.min(currentSetpoint + 1, 4);
  }
  public void decrementSetPoint() {
    currentSetpoint = Math.max(currentSetpoint - 1, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Setpoint", currentSetpoint);
  }
}