// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  private Joystick m_stick = new Joystick(Constants.STICK_ID);

  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private FeederSubsystem m_feederSubsystem = new FeederSubsystem();
  private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem(); 

  public DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem, 
    () -> { return m_stick.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS); }, 
    () -> { return m_stick.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS); }
  );
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_stick, Constants.Y_BUTTON).whenPressed(new InstantCommand(m_shooterSubsystem::toggleSolenoid));
    new JoystickButton(m_stick, Constants.B_BUTTON).whenPressed(new InstantCommand(m_shooterSubsystem::toggleShooter));
    new JoystickButton(m_stick, Constants.X_BUTTON).whenHeld(new StartEndCommand(
      () -> m_feederSubsystem.setMotors(Constants.FEEDER_FORWARD_SPEED),
      () -> m_feederSubsystem.setMotors(0.0), 
      m_feederSubsystem
    ));
    new JoystickButton(m_stick, Constants.A_BUTTON).whenHeld(new StartEndCommand(
      () -> m_feederSubsystem.setMotors(Constants.FEEDER_REVERSE_SPEED),
      () -> m_feederSubsystem.setMotors(0.0), 
      m_feederSubsystem
    ));
    new JoystickButton(m_stick, Constants.RIGHT_BUMPER).whenHeld(new StartEndCommand(
      () -> m_intakeSubsystem.setMotor(Constants.INTAKE_SPEED),
      () -> m_intakeSubsystem.setMotor(0.0),
      m_intakeSubsystem
    ));
    new JoystickButton(m_stick, Constants.LEFT_BUMPER).whenHeld(new StartEndCommand(
      () -> m_climberSubsystem.setMotors(Constants.CLIMBER_RAISE_SPEED),
      () -> m_climberSubsystem.setMotors(0.0),
      m_climberSubsystem
    ));
    new JoystickButton(m_stick, Constants.PREV_BUTTON).whenHeld(new InstantCommand(m_shooterSubsystem::stopShooter));
    new JoystickButton(m_stick, Constants.START_BUTTON).whenHeld(new StartEndCommand(
      () -> m_climberSubsystem.setMotors(Constants.CLIMBER_CLIMB_SPEED),
      () -> m_climberSubsystem.setMotors(0.0),
      m_climberSubsystem
    ));
    new Button(this::getLeftTrigger).whenHeld(new StartEndCommand(
      () -> m_intakeSubsystem.setExtender(Constants.INTAKE_EXTEND_SPEED),
      () -> m_intakeSubsystem.setExtender(0.0),
      m_intakeSubsystem
    ));
    new Button(this::getRightTrigger).whenHeld(new StartEndCommand(
      () -> m_intakeSubsystem.setExtender(Constants.INTAKE_RETRACT_SPEED),
      () -> m_intakeSubsystem.setExtender(0.0),
      m_intakeSubsystem
    ));
  }

  public boolean getLeftTrigger() {
    return m_stick.getRawAxis(Constants.LEFT_TRIGGER_AXIS) >= 0.95;
  }

  public boolean getRightTrigger() {
    return m_stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) >= 0.95;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return blank command: Instantly executes nothing and finishes
    return new InstantCommand();
  }
}
