package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.commands.Shoot;

import frc.robot.Constants.AutoConstants;

public class TrenchAuto extends SequentialCommandGroup implements Loggable{
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_robotDrive;
  private final IntakeSubsystem m_intake;
  private final ConveyorSubsystem m_conveyor;
  /**
   * Creates a new TrenchAuto.
   * @param shooter
   * @param robotDrive
   * @param intake
   * @param conveyor
   */
  public TrenchAuto(ShooterSubsystem shooter, DriveSubsystem robotDrive, IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    m_shooter = shooter;
    m_robotDrive = robotDrive;
    m_intake = intake;
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_robotDrive, m_intake, m_conveyor);

    // Commands to be run in the order they should be run in
    addCommands(
      //placed to face trench
      //start shooter to speed we want
      new InstantCommand(() -> {
        m_shooter.setSetpoint(AutoConstants.kTrenchAutoShootRPS);
        m_shooter.enable();
      }, m_shooter),

      //run conveyor when shooter is at speed (stop moving conveyor when not at speed)
      new Shoot(AutoConstants.kAutoShootTimeSeconds, m_shooter, m_conveyor),
      
      new RunCommand(() -> m_robotDrive.turnToAngle(180)),

      //lower intake and spin intake
      new InstantCommand(() -> {m_intake.toggleIntakePosition(true);
        m_intake.toggleIntakeWheels(true);}, m_intake),

      //drive forward distance of three balls (x feet)

      // Retract intake
      new InstantCommand(() -> {m_intake.toggleIntakePosition(true);
        m_intake.toggleIntakeWheels(true);}, m_intake),

      //turn around to face goal (0)
      new RunCommand(() -> m_robotDrive.turnToAngle(0)),

      //drive forward back to the line
      
      // Probably need to do a Limelight based AutoAim here but need to get it working first

      //start shooter to speed we want
      new InstantCommand(() -> {
        m_shooter.setSetpoint(AutoConstants.kTrenchAutoShootRPS);
        m_shooter.enable();
      }, m_shooter),

      //run conveyor when shooter is at speed (stop moving conveyor when not at speed)
      new Shoot(AutoConstants.kAutoShootTimeSeconds, m_shooter, m_conveyor),
      
      
      // Stop shooter
      new InstantCommand(() -> {
        m_shooter.setSetpoint(0);
        m_shooter.disable();
      }, m_shooter),

      //turn (-45) to pick up more balls
      new RunCommand(() -> m_robotDrive.turnToAngle(AutoConstants.kTrenchAutoShootAngle))

      // Drive some more down field
      //new DriveStraight(AutoConstants.kTrenchAutoDriveCenter, m_robotDrive)
    );
  }
}