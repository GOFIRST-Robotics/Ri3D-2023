// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToAprilTagCommand;

public class DriveToAprilTagAndPlaceAuto extends SequentialCommandGroup {
  /** Creates a new DriveToAprilTagAndPlace. */
  public DriveToAprilTagAndPlaceAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new DriveToAprilTagCommand(2, true));
  }
}
