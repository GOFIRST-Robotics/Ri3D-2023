// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {

  private DriveSubsystem m_drivetrainSubsystem;
  //private VisionSubsystem m_visionSubsystem;

  double angleToTarget;
  int targetTagID;
  double distanceToTarget;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTagCommand() {
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    //m_visionSubsystem = Robot.m_visionSubsystem;
    addRequirements(m_drivetrainSubsystem);//, m_visionSubsystem);
    angleToTarget = m_drivetrainSubsystem.getAngle() - 30;
    distanceToTarget = 1;
  }

  public DriveToAprilTagCommand(int targetTagID) {
    this();
    this.targetTagID = targetTagID; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("STARTED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angleToTarget = m_visionSubsystem.getBestTarget().getYaw();
    double rotationalError = angleToTarget - m_drivetrainSubsystem.getAngle();
    PhotonTrackedTarget trackedTarget;
    // if(targetTagID == 0) {
    //   trackedTarget = m_visionSubsystem.getBestTarget();
    // } else {
    //   trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
    // }
    // double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
    //   Constants.CAMERA_HEIGHT_METERS,
    //   Constants.TARGET_HEIGHT_METERS,
    //   Constants.CAMERA_PITCH_RADIANS,
    //   Units.degreesToRadians(trackedTarget.getPitch())
    // );
    double distanceToTarget = this.distanceToTarget -= 0.05;
    double TARGET_DISTANCE_TO_TARGET_METERS = 0.5;
    double translationalError = distanceToTarget - TARGET_DISTANCE_TO_TARGET_METERS;
    double trasnlationValue = translationalError * Constants.TRACKED_TAG_DRIVE_KP;
    double rotationValue = rotationalError * Constants.GYRO_KP;
    double value1 = trasnlationValue + rotationValue;
    double value2 = trasnlationValue - rotationValue;
    double leftDriveRate;
    double rightDriveRate;
    if (value1 > 1 || value2 > 1) {
      double max = Math.max(Math.abs(value1), Math.abs(value2));
      leftDriveRate = Math.copySign(value1/max, value1);
      rightDriveRate = Math.copySign(value2/max, value1);
    } else {
      leftDriveRate = value1;
      rightDriveRate = value2;
    }
    m_drivetrainSubsystem.drive(leftDriveRate, rightDriveRate);
    System.out.println("Distance: " + distanceToTarget);
    System.out.println("Rotational Error: " + rotationalError);
    System.out.println("Translational Error: " + translationalError);
    System.out.println("Rotational Value: " + rotationValue);
    System.out.println("Translational Value: " + trasnlationValue);
    System.out.println("leftDriveRate: " + leftDriveRate);
    System.out.println("rightDriveRate: " + rightDriveRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDED");
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distanceToTarget <= 0.5;
  }
}
