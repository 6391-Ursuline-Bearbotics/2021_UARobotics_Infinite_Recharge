// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  ShooterSubsystem m_shooter;
  ConveyorSubsystem m_conveyor;
  Double timeout;
  Double startTime;

  public AutoShoot(ShooterSubsystem shooter, ConveyorSubsystem conveyor, Double timeoutSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_conveyor = conveyor;
    timeout = timeoutSeconds;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    // Make sure the shooter is spinning start it if not
    if (m_shooter.getSetpoint() == 0) {
      m_shooter.setSetpoint(AutoConstants.kAutoShootRPS);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.atSetpoint() || m_conveyor.getTopConveyor()) {
      m_conveyor.turnOn();
    }
    else {
      m_conveyor.turnOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("end time", startTime);
    return (Timer.getFPGATimestamp() - startTime) > timeout;
  }
}
