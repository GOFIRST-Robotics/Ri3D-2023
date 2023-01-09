// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  private boolean isExtended;
  private DoubleSolenoid grabberSolenoid;

  /** Subsystem for controlling the Grabber */
  public GrabberSubsystem() {
    grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GRABBER_SOLENOID_1_ID, Constants.GRABBER_SOLENOID_2_ID);
    isExtended = false;
  }

  public void extend() {
    grabberSolenoid.set(Value.kReverse);
    isExtended = true;
  }

  public void retract() {
    grabberSolenoid.set(Value.kForward);
    isExtended = false;
  }

  public void toggle() {
    if (isExtended) {
      retract();
    } else {
      extend();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}