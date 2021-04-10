// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  DriveSubsystem m_robotDrive;
  PhotonVision m_PhotonVision;
  ShooterSubsystem m_shooter;
  DoubleSupplier m_joystickY;

  public AutoAim(DriveSubsystem m_drive, PhotonVision PhotonVision, ShooterSubsystem shooter, DoubleSupplier joystickY) {
    m_robotDrive = m_drive;
    m_PhotonVision = PhotonVision;
    m_shooter = shooter;
    m_joystickY = joystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PhotonVision.lightsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the latest result from the Limelight Camera
    var result = m_PhotonVision.m_limePhoton.getLatestResult();

    // Make sure it has a target we can use
    SmartDashboard.putBoolean("hastargets", result.hasTargets());
    if (result.hasTargets()) {
      // Get the angle of the best target needs to be inverted because Photon is + right
      var targetAngle = -result.getBestTarget().getYaw();
      SmartDashboard.putNumber("targetangle", targetAngle);

      // Drive with the speed of the joystick to the angle of the best target
      m_robotDrive.driveToTarget(m_joystickY.getAsDouble(), targetAngle);

      // Get the distance to the Target
      double range = PhotonUtils.calculateDistanceToTargetMeters(LimelightConstants.kCameraHeight, LimelightConstants.kTargetHeight,
          Units.degreesToRadians(LimelightConstants.kCameraAngle), Units.degreesToRadians(result.getBestTarget().getPitch()));
      SmartDashboard.putNumber("distanceToTarget", Units.metersToFeet(range));

      // this should actually be a conversion from range to speed
      double speedRPS = m_shooter.getMeasurement();

      // set shooter speed to value based on distance
      m_shooter.setSetpoint(speedRPS);

      // should already be enabled and up to speed by operator but just in case
      m_shooter.enable();
    }
    else {
      // If we don't have a target stop since we don't know where we are going possibly should use regular arcade here?
      m_robotDrive.stopmotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PhotonVision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
