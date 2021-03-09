package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StealAuto extends SequentialCommandGroup {
    public StealAuto(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor) {        
        // Loads all of the trajectories we will need.  This happens on init so we get it out of the way before actually running.
        Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Steal1");
        
        addCommands(
            // Resets odometery which is what defines where the robot is.  This isn't generally nessecary for a match auto because the
            // robot will start on the field at the right spot (hopefully) but it is useful for testing / simulation.    
            new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),

            // Turn the shooter on and give it time to get up to speed.
            new InstantCommand(() -> m_shooter.setSetpoint(AutoConstants.kAutoShootRPS)),
            
            // Lowers the intake and turns the wheels on
            new InstantCommand(() -> m_intake.deployIntake()),

            // Runs the Steal1 trajectory that grabs 2 balls in their trench and then goes across the field to our shooting spot.
            m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Steal1"),

            // Shoot all 5 balls that we have collected (will probably be just 3 for other autos)
            new AutoShoot(m_shooter, AutoConstants.kAutoShoot5)
        );
    }
}