// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  DriveSubsystem m_robotDrive;
  PhotonVision m_PhotonVision;

  public AutoAim(DriveSubsystem m_drive, PhotonVision PhotonVision) {
    m_robotDrive = m_drive;
    m_PhotonVision = PhotonVision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PhotonVision.lightsOn();
    m_robotDrive.driveToTarget(() -> 0).withTimeout(3.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PhotonVision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
