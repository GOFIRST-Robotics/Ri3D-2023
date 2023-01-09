// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController;

  /** Subsystem for controlling RGB LED strips using a REV Blinkin */
  public LEDSubsystem() {
    ledPWMController = new Spark(Constants.LED_PWM_ID);
  }

  public void setLEDMode(LEDMode ledMode) {
    ledPWMController.set(ledMode.pwmSignal);
  }
  public void setLEDPWM(double PWM) {
    ledPWMController.set(PWM);
  }

  public enum LEDMode {
    GREEN(0.77),
    WHITE(0.93);
  
    public double pwmSignal;
  
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}