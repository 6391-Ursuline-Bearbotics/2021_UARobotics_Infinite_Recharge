package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterAuto extends SequentialCommandGroup {
  public CenterAuto(DriveSubsystem m_robotDrive) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Center1");
      Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Center2");
      Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Center3");
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Center1"),
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Center2"),
          m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Center3")
      );
  }
}