// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.autonomous.BalanceBeamAutonomous;
import frc.robot.commands.autonomous.Drive1MeterAuto;
import frc.robot.commands.autonomous.PlaceCubeAutonomous;
import frc.robot.commands.autonomous.AutonomousMode_Default;
import frc.robot.commands.autonomous.SquareAutonomous;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.BalanceOnBeamCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveInFrontOfTag;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.ExtenderControlCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CommandBase m_autonomousCommand;
	SendableChooser<CommandBase> autonChooser = new SendableChooser<CommandBase>(); // Create a chooser to select an autonomous command

  public static SendableChooser<Boolean> toggleExtenderPID = new SendableChooser<Boolean>(); // Create a chooser to toggle whether the extender default command should run

  public static final GenericHID controller = new GenericHID(Constants.USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  public static final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem(); // Grabs both cubes and cones
  public static final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem(); // Used to reach out and score with the grabber
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Subsystem for interacting with Photonvision
  public static final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); // Subsytem for controlling the REV Blinkin LED module
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    configureButtonBindings(); // Configure the button bindings

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		autonChooser.addOption("Balance Beam Auto", new BalanceBeamAutonomous());
		autonChooser.addOption("Place Object Auto", new PlaceCubeAutonomous());
    autonChooser.addOption("Square Auto", new SquareAutonomous());
    autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
		SmartDashboard.putData("Auto Mode", autonChooser);

    // Add chooser options for toggling the Extender default command on/off //
    toggleExtenderPID.setDefaultOption("OFF", false);
    toggleExtenderPID.addOption("ON", true);

    SmartDashboard.putData("Extender PID Control", toggleExtenderPID);

    m_driveSubsystem.setDefaultCommand(new DriveCommand());
    m_extenderSubsystem.setDefaultCommand(new ExtenderControlCommand());

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
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
    SmartDashboard.putNumber("Extender Position", m_extenderSubsystem.getEncoderPosition());
    //if (m_visionSubsystem.getHasTarget()) {System.out.println("TARGET SKEW: " + m_visionSubsystem.getBestTarget().getBestCameraToTarget().getRotation().getZ());}
     //System.out.println(m_driveSubsystem.getEncoderRate());
    // if (m_visionSubsystem.getHasTarget()) {
    //  System.out.println("tagDistance" + m_visionSubsystem.getBestTarget().getBestCameraToTarget().getTranslation().getX());
    //  double reportedTagZAngle = Math.toDegrees(m_visionSubsystem.getBestTarget().getBestCameraToTarget().getRotation().getZ());
    //  System.out.println("reported Tag z Angle" + reportedTagZAngle);
    //  System.out.println("Tag Z Angle" + Math.copySign(180 - Math.abs(reportedTagZAngle), reportedTagZAngle));
    // }
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
    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
    m_extenderSubsystem.resetEncoder();
    

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_extenderSubsystem.resetEncoder();
    m_driveSubsystem.resetEncoders();
    m_LEDSubsystem.setLEDMode(LEDMode.GREEN);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.print("Has target? ");
    // System.out.println(m_visionSubsystem.getHasTarget());
    // System.out.print("Best target's angle from the robot: ");
    // System.out.println(m_visionSubsystem.getBestTarget().getYaw());
    // if (m_visionSubsystem.getHasTarget()) {
    //   System.out.println(PhotonUtils.calculateDistanceToTargetMeters(
    //     Constants.CAMERA_HEIGHT_METERS, 
    //     Constants.TARGET_HEIGHT_METERS, 
    //     Constants.CAMERA_PITCH_RADIANS, 
    //     Units.degreesToRadians(m_visionSubsystem.getBestTarget().getPitch())
    //   ));
    // }
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
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> controller.getRawButton(Constants.LEFT_BUMPER)).onTrue(new InstantCommand(() -> m_grabberSubsystem.toggle()));
    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new InstantCommand(() -> m_extenderSubsystem.toggleExtenderRaiser()));

    // Extender Controls //
    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(0)));
    new POVButton(controller, 180).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(1)));
    new POVButton(controller, 90).onTrue(new InstantCommand(() -> m_extenderSubsystem.incrementSetPoint()));
    new POVButton(controller, 0).onTrue(new InstantCommand(() -> m_extenderSubsystem.changeSetpoint(4)));
    new POVButton(controller, 270).onTrue(new InstantCommand(() -> m_extenderSubsystem.decrementSetPoint()));

    // Drivetrain Controls //
    new Trigger(() -> controller.getRawButton(Constants.Y_BUTTON)).onTrue(new DriveToAprilTagCommand(2.5, true));
    new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).whileTrue(new BalanceOnBeamCommand());
    new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).whileTrue(new DriveInFrontOfTag(0.3));
  }
}