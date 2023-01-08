// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  private VictorSP m_leftFrontMotor;
  private VictorSP m_rightFrontMotor;
  private VictorSP m_leftRearMotor;
  private VictorSP m_rightRearMotor;
  
  private AHRS navx = new AHRS(SerialPort.Port.kUSB);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftFrontMotor = new VictorSP(Constants.LEFT_FRONT_DRIVE_MOTOR_ID);
    m_rightFrontMotor = new VictorSP(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID);
    m_leftRearMotor = new VictorSP(Constants.LEFT_REAR_DRIVE_MOTOR_ID);
    m_rightRearMotor = new VictorSP(Constants.RIGHT_REAR_DRIVE_MOTOR_ID);

    m_leftFrontMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightFrontMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
    m_leftRearMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightRearMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
  }

  public void drive(double leftPct, double rightPct) {
    m_leftFrontMotor.set(leftPct);
    m_leftRearMotor.set(leftPct);
    m_rightFrontMotor.set(rightPct);
    m_rightRearMotor.set(rightPct);
  }

  public void calibrateGyro() {
    navx.calibrate();
  }

  public void zeroGyro() {
    System.out.println(navx.isConnected());
    System.out.println(navx.isMagnetometerCalibrated());
    navx.zeroYaw();
  }
  
  public double getYaw() {
    return navx.getYaw();
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getAngle() {
    return navx.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set the default command for the subsystem

}