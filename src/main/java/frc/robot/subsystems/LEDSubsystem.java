// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController; // The REV Blinkin module acts like a Spark motor controller (it can be controlled over PWM)

  /** Subsystem for controlling RGB LED strips using a REV Blinkin */
  public LEDSubsystem() {
    ledPWMController = new Spark(Constants.LED_PWM_ID); // Declare the PWM port it is wired to
  }

  public void setLEDMode(LEDMode ledMode) { // Set a specific LED mode (these modes are presets declared in the enumerator below)
    ledPWMController.set(ledMode.pwmSignal);
  }
  
  public void setLEDPWM(double PWM) { // Set the raw PWM signal
    ledPWMController.set(PWM);
  }

  // Declare the preset LED modes we want to use
  public enum LEDMode {
    GREEN(0.77),
    WHITE(0.93);
  
    public double pwmSignal;
  
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}