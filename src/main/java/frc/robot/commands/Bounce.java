package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Bounce extends SequentialCommandGroup {
  public Bounce(DriveSubsystem m_robotDrive) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Bounce1");
      Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Bounce2");
      Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Bounce3");
      Trajectory trajectory4 = m_robotDrive.loadTrajectoryFromFile("Bounce4");
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Bounce1"),
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Bounce2"),
          m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Bounce3"),
          m_robotDrive.createCommandForTrajectory(trajectory4, false).withTimeout(50).withName("Bounce4")
      );
  }
}