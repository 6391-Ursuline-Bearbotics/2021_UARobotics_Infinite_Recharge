// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MaintainHeadingConstants;
import frc.robot.UA6391.Xbox6391;
import frc.robot.subsystems.DriveSubsystem;

public class MaintainHeading extends CommandBase {
  private final DriveSubsystem m_robotDrive;
  private final Xbox6391 drv;
  private double initialHeading;
  private PIDController pid;

  /** Creates a new MaintainHeading. */
  public MaintainHeading(Xbox6391 drvIn, DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
    drv = drvIn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialHeading = m_robotDrive.getHeading();
    pid = new PIDController(MaintainHeadingConstants.kP, MaintainHeadingConstants.kI, MaintainHeadingConstants.kD);
    pid.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rot = pid.calculate(m_robotDrive.getHeading(), initialHeading);
    m_robotDrive.arcadeDrive(drv.JoystickLY(), rot);
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
