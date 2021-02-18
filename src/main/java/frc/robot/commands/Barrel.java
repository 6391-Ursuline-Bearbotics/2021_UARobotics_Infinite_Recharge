package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Barrel extends SequentialCommandGroup {
  public Barrel(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig config = new TrajectoryConfig(2, 3);
      
      Trajectory trajectory1 = m_robotDrive.generateTrajectory("Barrel1", config);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Barrel1")
      );
  }
}