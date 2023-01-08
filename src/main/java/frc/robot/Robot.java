// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.AutonomousMode_1;
import frc.robot.commands.autonomous.AutonomousMode_2;
import frc.robot.commands.autonomous.AutonomousMode_Default;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.ExtenderMoveToSetpoint;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CommandBase m_autonomousCommand;
	SendableChooser<CommandBase> chooser = new SendableChooser<CommandBase>();

  public static final GenericHID controller = new GenericHID(Constants.USB_PORT_ID);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  public static final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem();
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public static final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    configureButtonBindings(); // Configure the button bindings

    // Autonomous Routines //
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		chooser.addOption("Custom Auto 1", new AutonomousMode_1());
		chooser.addOption("Custom Auto 2", new AutonomousMode_2());

    m_driveSubsystem.setDefaultCommand(new DriveCommand());
    m_extenderSubsystem.setDefaultCommand(new ExtenderMoveToSetpoint());
				
		SmartDashboard.putData("Auto Mode", chooser);

    m_driveSubsystem.calibrateGyro();
    m_extenderSubsystem.resetEncoder();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Extender Position", m_extenderSubsystem.currentSetpoint);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    System.out.println("ATUO STARTED");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_driveSubsystem.zeroGyro();
    m_LEDSubsystem.setLEDMode(LEDMode.GREEN);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.print("Has target? ");
    // System.out.println(m_visionSubsystem.getHasTarget());
    // System.out.print("Best target's angle from the robot: ");
    // System.out.println(m_visionSubsystem.getBestTarget().getYaw());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).onTrue(new InstantCommand(() -> m_grabberSubsystem.toggle()));

    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0)));
    new POVButton(controller, 180).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(1)));
    new POVButton(controller, 90).onTrue(new InstantCommand(() -> m_extenderSubsystem.incrementSetPoint()));
    new POVButton(controller, 0).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4)));
    new POVButton(controller, 270).onTrue(new InstantCommand(() -> m_extenderSubsystem.decrementSetPoint()));

    new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).whileTrue(new BalanceOnBeamCommand());
  }

  public boolean getLeftTrigger() {
    return controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS) >= 0.95;
  }

  public boolean getRightTrigger() {
    return controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS) >= 0.95;
  }
}