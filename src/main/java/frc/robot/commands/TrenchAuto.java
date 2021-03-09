package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrenchAuto extends SequentialCommandGroup {
    public TrenchAuto(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor) {        
        // Loads all of the trajectories we will need.  This happens on init so we get it out of the way before actually running.
        Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Center1");
        Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Trench2");
        Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Trench3");
      
        addCommands(
            // Resets odometery which is what defines where the robot is.  This isn't generally nessecary for a match auto because the
            // robot will start on the field at the right spot (hopefully) but it is useful for testing / simulation.
            new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),
            
            m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Center1"),
            m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Trench2"),
            m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Trench3")
      );
  }
}