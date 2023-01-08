// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  private Solenoid grabberSolenoid;
  private boolean isExtended;

  /** Creates a new Grabber Subsystem. */
  public GrabberSubsystem() {
    grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.GRABBER_SOLENOID_ID);
    isExtended = false;
  }

  public void extend() {
    grabberSolenoid.set(true);
    isExtended = true;
  }

  public void retract() {
    grabberSolenoid.set(false);
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