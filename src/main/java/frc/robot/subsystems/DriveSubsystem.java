// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.subsystems;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  // Drivetrain Motor Controllers
  private VictorSP m_leftFrontMotor;
  private VictorSP m_rightFrontMotor;
  private VictorSP m_leftRearMotor;
  private VictorSP m_rightRearMotor;

  // Drivetrain Encoders
  private Encoder leftDriveEncoder;
  private Encoder rightDriveEncoder;

  // These objects help with fancy path following
  private DifferentialDriveOdometry odometry;
  private DifferentialDriveKinematics kinematics;

  private int direction = 1; // This variable is here because we wanted to be able to flip which side of the robot is the 'front'
  
  private AHRS navx = new AHRS(SerialPort.Port.kUSB); // Instantiate a NavX Gyroscope

  // Create a chooser for selecting the desired drive speed scale
  SendableChooser<Double> driveScaleChooser = new SendableChooser<Double>();
  public double CURRENT_DRIVE_SCALE;

  /** Subsystem for controlling the Drivetrain and accessing the NavX Gyroscope */
  public DriveSubsystem() {
    // Instantiate the Drivetrain motor controllers
    m_leftFrontMotor = new VictorSP(Constants.LEFT_FRONT_DRIVE_MOTOR_ID);
    m_rightFrontMotor = new VictorSP(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID);
    m_leftRearMotor = new VictorSP(Constants.LEFT_REAR_DRIVE_MOTOR_ID);
    m_rightRearMotor = new VictorSP(Constants.RIGHT_REAR_DRIVE_MOTOR_ID);

    // Reverse some of the motors if needed
    m_leftFrontMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightFrontMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);
    m_leftRearMotor.setInverted(Constants.DRIVE_INVERT_LEFT);
    m_rightRearMotor.setInverted(Constants.DRIVE_INVERT_RIGHT);

    // Instantiate the drivetrain encoders and set the distancePerPulse
    leftDriveEncoder = new Encoder(Constants.LEFT_ENCODER_CHANNEL_A, Constants.LEFT_ENCODER_CHANNEL_B);
    rightDriveEncoder = new Encoder(Constants.RIGHT_ENCODER_CHANNEL_A, Constants.RIGHT_ENCODER_CHANNEL_B, true);
    leftDriveEncoder.setDistancePerPulse(Constants.WHEEL_CIRCUMFERENCE / Constants.LEFT_ENCODER_COUNTS_PER_REV);
    leftDriveEncoder.setDistancePerPulse(Constants.WHEEL_CIRCUMFERENCE / Constants.RIGHT_ENCODER_COUNTS_PER_REV);

    resetEncoders(); // Zero the encoders

    // These objects help with fancy path following
    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftDistance(), getRightDistance());
    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);

    // Drive Scale Options //
    driveScaleChooser.addOption("100%", 1.0);
    driveScaleChooser.setDefaultOption("75%", 0.75);
    driveScaleChooser.addOption("50%", 0.5);
    driveScaleChooser.addOption("25%", 0.25);

    SmartDashboard.putData("Drivetrain Speed", driveScaleChooser);
  }

  /* Set power to the drivetrain motors */
  public void drive(double leftPercentPower, double rightPercentPower) {
    m_leftFrontMotor.set(direction * leftPercentPower);
    m_leftRearMotor.set(direction * leftPercentPower);
    m_rightFrontMotor.set(direction * rightPercentPower);
    m_rightRearMotor.set(direction * rightPercentPower);
  }
  public void stop() {
    drive(0, 0);
  }

  // NavX Gyroscope Methods //
  public void calibrateGyro() {
    navx.calibrate();
  }
  public void zeroGyro() {
    System.out.println("NavX Connected: " + navx.isConnected());
    navx.reset();
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

  // Speed will be measured in meters/second
  public double getLeftSpeed() {
    return leftDriveEncoder.getRate() / 1000; // Multiply by 1000 to convert from millimeters to meters
  }
  public double getRightSpeed() {
    return rightDriveEncoder.getRate() / 1000; // Multiply by 1000 to convert from millimeters to meters
  }
  public double getAverageEncoderSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  // Distance will be measured in meters
  public double getLeftDistance() {
    return leftDriveEncoder.getDistance() / 1000; // Multiply by 1000 to convert from millimeters to meters
  }
  public double getRightDistance() {
    return rightDriveEncoder.getDistance() / 1000; // Multiply by 1000 to convert from millimeters to meters
  }
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  // Zero the drivetrain encoders
  public void resetEncoders() {
		leftDriveEncoder.reset();
    rightDriveEncoder.reset();
	}

  // These return values are measured in raw encoder counts
  public double getLeftRaw() {
    return leftDriveEncoder.get();
  }
  public double getRightRaw() {
    return rightDriveEncoder.get();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void updateOdometry() {
    odometry.update(navx.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  // Returns the current wheel speeds of the robot.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightDistance());
  }

  // These methods allow us to flip which side is considered the "front" of the robot
  public void setDirection(DIRECTION direction) {
    this.direction = direction.direction;
  }
  public void toggleDirection() {
    this.direction *= -1;
  }

  @Override
  public void periodic() {
    updateOdometry(); // Update our position on the field

    CURRENT_DRIVE_SCALE = driveScaleChooser.getSelected(); // Continously update the desired drive scale

    SmartDashboard.putNumber("Left Drive Encoder", leftDriveEncoder.getRaw()); // Publish raw encoder data to Shuffleboard
    SmartDashboard.putNumber("Right Drive Encoder", rightDriveEncoder.getRaw()); // Publish raw encoder data to Shuffleboard
  }

  // Helps with flipping which side is considered the "front" of the robot
  enum DIRECTION {
    INTAKE_FRONT(1),
    EXTENDER_FRONT(-1);

    public int direction;

    DIRECTION(int direction) {
      this.direction = direction;
    }
  }
}