// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_leftSupplier;
  private DoubleSupplier m_rightSupplier;

  // The default tank drive mechanism
  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
    addRequirements(subsystem);
    m_subsystem = subsystem;
    
    m_leftSupplier = left;
    m_rightSupplier = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(m_leftSupplier.getAsDouble(), m_rightSupplier.getAsDouble());
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
