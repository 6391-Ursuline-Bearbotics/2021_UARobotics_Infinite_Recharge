package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveStraight extends CommandBase implements Loggable{
  private final DriveSubsystem m_robotDrive;

  private final Double distance;
  /**
   * Creates a new AutoAimCommand.
   * @param m_robotDrive The subsystem used by this command.
   */
  public DriveStraight(double distanceIn, DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
    distance = distanceIn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.distancesetup();
    m_robotDrive.drivePositionGyro(distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  @Log
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_robotDrive.atSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.arcadeDrive(0,0);
  }
}