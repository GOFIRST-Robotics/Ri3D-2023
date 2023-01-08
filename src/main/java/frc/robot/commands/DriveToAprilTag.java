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
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTag extends CommandBase {

  private DriveSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  double angleToTarget;
  int targetTagID;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag() {
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
  }

  public DriveToAprilTag(int targetTagID) {
    this();
    this.targetTagID = targetTagID; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleToTarget = m_visionSubsystem.getBestTarget().getYaw();
    double rotationalError = angleToTarget - m_drivetrainSubsystem.getAngle();
    PhotonTrackedTarget trackedTarget;
    if(targetTagID == 0) {
      trackedTarget = m_visionSubsystem.getBestTarget();
    } else {
      trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
    }
    double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
      Constants.CAMERA_HEIGHT_METERS,
      Constants.TARGET_HEIGHT_METERS,
      Constants.CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(trackedTarget.getPitch())
    );
    double TARGET_DISTANCE_TO_TARGET_METERS = 0.5;
    double translationalError = distanceToTarget - TARGET_DISTANCE_TO_TARGET_METERS;
    double leftDriveRate = Math.min(translationalError * Constants.TRACKED_TAG_DRIVE_KP + rotationalError * Constants.GYRO_KP, 1);
    double rightDriveRate = Math.min(translationalError * Constants.TRACKED_TAG_DRIVE_KP - rotationalError * Constants.GYRO_KP, 1);
    m_drivetrainSubsystem.drive(leftDriveRate, rightDriveRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
