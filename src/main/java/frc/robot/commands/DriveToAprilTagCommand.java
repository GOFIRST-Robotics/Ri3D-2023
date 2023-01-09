// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {

  private DriveSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  double angleToTarget;
  int targetTagID;
  double distanceToTarget;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTagCommand() {
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
  }

  public DriveToAprilTagCommand(int targetTagID) {
    this();
    this.targetTagID = targetTagID; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.getHasTarget()) {
      PhotonTrackedTarget trackedTarget;
      if(targetTagID == 0) {
        trackedTarget = m_visionSubsystem.getBestTarget();
      } else {
        trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
      }
      double rotationalError = trackedTarget.getYaw();      
      double DESIRED_TARGET_AREA = 2.5;
      double translationalError = DESIRED_TARGET_AREA - trackedTarget.getArea();
      double translationValue = translationalError * 0.2;
      double rotationValue = -rotationalError * Constants.GYRO_KP*2.5;
      double value1 = translationValue - rotationValue;
      double value2 = translationValue + rotationValue;
      double leftDriveRate;
      double rightDriveRate;
      if (value1 > Constants.POWER_CAP || value2 > Constants.POWER_CAP) {
        double max = Math.max(Math.abs(value1), Math.abs(value2));
        leftDriveRate = Math.copySign(value1/max, value1);
        rightDriveRate = Math.copySign(value2/max, value1);
      } else {
        leftDriveRate = value1;
        rightDriveRate = value2;
      }
      m_drivetrainSubsystem.drive(leftDriveRate, rightDriveRate);
      System.out.println("Distance: " + distanceToTarget); // debug data
      System.out.println("Rotational Error: " + rotationalError);
      System.out.println("Translational Error: " + translationalError);
      System.out.println("Rotational Value: " + rotationValue);
      System.out.println("Translational Value: " + translationValue);
      System.out.println("leftDriveRate: " + leftDriveRate);
      System.out.println("rightDriveRate: " + rightDriveRate);
    } else {
      m_drivetrainSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return distanceToTarget <= 0.5;
  }
}