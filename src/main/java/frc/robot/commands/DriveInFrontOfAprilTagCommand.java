// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveInFrontOfAprilTagCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private boolean foundTarget;
  private double tagDistance;
  private double tagZAngle;
  private double distanceToDrive;
  private double angleToDriveIn;
  private double targetDistanceInFrontOfTag;
  private double initialDistance;
  private double initialAngle;
  private DRIVE_STATE driveState;
  private boolean done;

  // FIXME: This code has not been tested much and does NOT work how we want it to yet; use this code with caution

  /** Uses fancy math with the extpected distance and z rotation of a visible
   *  apriltag to determine a desired point right in front of the tag and drive
   * there
   * */
  public DriveInFrontOfAprilTagCommand(double targetDistanceInFrontOfTag) {
    m_driveSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    this.targetDistanceInFrontOfTag = targetDistanceInFrontOfTag;
    addRequirements(m_driveSubsystem, m_visionSubsystem);
    
    // Debug
    System.out.println("COMMAND INSTANTIATED");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveState = DRIVE_STATE.ROTATING;
    
    // Debug
    System.out.println("COMMAND STARTING");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Debug
    System.out.println("COMMAND LOOP");

    // Will only run once after the vision subsystem has a target
    if (!foundTarget && m_visionSubsystem.getHasTarget()) {
      // Big math section, which in short uses the 3d pose estimation and rotation data reported by photon vision about
      // the apriltag in view, and takes the position of the robot being at the origin of a 2d graph facing along the y-axis
      // and the april tag being the y interncept of a line that points along the direciton the tag faces. This finds the slope
      // of that line, the coordinates of a point a specified distance from the y intercept along the path of the line using
      // vector math, and determines the distance the robot needs to drive and rotation it has to turn to get there.
      Transform3d targetPose = m_visionSubsystem.getBestTarget().getBestCameraToTarget();
      tagDistance = targetPose.getTranslation().getX();
      double reportedTagZAngle = -Math.toDegrees(targetPose.getRotation().getZ());
      tagZAngle = Math.copySign(180 - Math.abs(reportedTagZAngle), reportedTagZAngle);
      double x2d = Math.toDegrees(Math.sin(Math.toRadians(90 - tagZAngle))) / (tagDistance * Math.toDegrees(Math.sin(Math.toRadians(tagZAngle)))); // Get a 2d realtive to origin x translation to target using law of sines
      double equationSlope = -tagDistance / x2d; // Get the equaiton of the slope of the line through the april tag
      double targetX = targetDistanceInFrontOfTag * (1 / Math.sqrt(1 + equationSlope * equationSlope)); // get Y position by solving vector equation (1/sqrt is getting unit vector)
      double targetY = tagDistance + targetDistanceInFrontOfTag * (equationSlope / Math.sqrt(1 + equationSlope * equationSlope)); // get Y position
      // Finding distance and rotation to drive in wouldn't be neccessary if we had trajectory command availible, but we didn't have that up and running due to sysid issues (the tool didn't work)
      distanceToDrive = Math.hypot(targetX, targetY); // Hypotenuse of target X and Y to find driving distance;
      double division = targetY / targetX;
      angleToDriveIn = 90 - Math.toDegrees(Math.atan(division));
      initialDistance = m_driveSubsystem.getLeftDistance();
      initialAngle = m_driveSubsystem.getAngle();
      foundTarget = true;

      // Debug
      System.out.println("Tag Distance" + tagDistance);
      System.out.println("TagZAngle " + tagZAngle);
      System.out.println("x2d " + x2d);
      System.out.println("equationSlope" + equationSlope);
      System.out.println("Target X" + targetX);
      System.out.println("Target Y " + targetY);
      System.out.println("Distance to drive " + distanceToDrive);
      System.out.println("Division " + division);
      System.out.println("Geater than +- pi/2?" + Math.asin(division));
      System.out.println("degrees " + Math.toDegrees(Math.asin(division)));
      System.out.println("Angle to drive in " + angleToDriveIn);
      System.out.println("Initiail Distance " +initialDistance);
   }

   if (foundTarget) {
    // Debug
    System.out.println("FOUND TARGET");

    // Quick state machine to get the robot to the desired points, bascially uses what as already written in other drive commands
    switch (driveState) {
      case ROTATING:
        System.out.println("COMMAND ROTATING");
        double error = initialAngle + angleToDriveIn - m_driveSubsystem.getAngle();
        double value = error * Constants.GYRO_TURN_KP; // Multiply by scaling factor kp to determine motor percent power between 1 and 100 percent
        if (Math.abs(value) > 0.75) {
          value = Math.copySign(0.75, value);
        }
        if (Math.abs(value) < 0.15) { // Min drive value
          value = Math.copySign(0.15, value);
        }
        m_driveSubsystem.drive(value, -value); // write calculated values to m_DriveSubsystem

        if (error < Constants.DRIVE_TURNING_THRESHOLD_DEGREES) {
          driveState = DRIVE_STATE.DRIVING;
        }

        // Debug
        System.out.println(m_driveSubsystem.getAngle());
        System.out.println("error" + error);
        System.out.println("target" + angleToDriveIn);
        System.out.println("degrees to turn" + angleToDriveIn);
        System.out.println("value" + value);
      break;
      case DRIVING:
        System.out.println("COMMAND DRIVING");
        error = initialDistance + distanceToDrive - m_driveSubsystem.getRightDistance();
        double powerPct = error * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP;
        if (powerPct > Constants.APRILTAG_POWER_CAP) {
          powerPct = Constants.APRILTAG_POWER_CAP;
        } else if (powerPct < -Constants.APRILTAG_POWER_CAP) {
          powerPct = -Constants.APRILTAG_POWER_CAP;
        }
        m_driveSubsystem.drive(powerPct, powerPct);

        if (error < 0.1) {
          done = true;
        }

        // Debug
        System.out.println("ERROR " + error);
        System.out.println("INITIAL DISTANCE: " + initialDistance);
        System.out.println("GOAL DISTANCE: " + (distanceToDrive + initialDistance));
        System.out.println("CURRENT DISTANCE: " + m_driveSubsystem.getRightDistance());
        System.out.println("POWER: " + powerPct);
      break;
      default:
      break;
    }
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();

    System.out.println("COMMAND ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

  enum DRIVE_STATE {
    ROTATING,
    DRIVING
  }
}