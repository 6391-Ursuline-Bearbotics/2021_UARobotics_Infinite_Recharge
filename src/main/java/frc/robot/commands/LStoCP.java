package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LStoCP extends SequentialCommandGroup {
  public LStoCP(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake) {        
    Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("LS to CP");
      
    addCommands(
      // Resets odometery which is what defines where the robot is.  This isn't generally nessecary for a match auto because the
      // robot will start on the field at the right spot (hopefully) but it is useful for testing / simulation.    
      new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),

      // Turn on Shooter
      new InstantCommand(() -> {
        m_shooter.setSetpoint(ShooterConstants.kShooterFarTrenchRPS);
        m_shooter.enable();
      }, m_shooter),

      // Retract the intake so it doesn't get hit
      new InstantCommand(() -> m_intake.retractIntake()),

      // Run the trajectory driving us to the control panel
      m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("LS to CP")
    );
  }
}