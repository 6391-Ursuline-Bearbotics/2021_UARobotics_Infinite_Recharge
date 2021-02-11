package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveStraight extends CommandBase implements Loggable{
  private final DriveSubsystem m_robotDrive;
  private final WPI_TalonSRX m_talonsrxright;
  private final Double target_sensorUnits;

  private final Double distance;
  /**
   * Creates a new AutoAimCommand.
   * @param m_robotDrive The subsystem used by this command.
   */
  public DriveStraight(double distanceIn, DriveSubsystem robotDrive) {
    m_robotDrive = robotDrive;
    distance = distanceIn;
    m_talonsrxright = m_robotDrive.getRightMaster();
    target_sensorUnits = (distanceIn * DriveConstants.kEncoderCPR) / DriveConstants.kWheelCircumferenceInches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.distancesetup();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, m_talonsrxright.getSelectedSensorPosition(1));
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