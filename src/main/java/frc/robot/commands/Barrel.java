package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Barrel extends SequentialCommandGroup {
  public Barrel(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig config = new TrajectoryConfig(2.2, 3);
      config.setEndVelocity(5);
      TrajectoryConfig config2 = new TrajectoryConfig(3.5, 5);
      config2.setStartVelocity(5);
      
      Trajectory trajectory1 = m_robotDrive.generateTrajectory("Barrel1", config);
      Trajectory trajectory2 = m_robotDrive.generateTrajectory("Barrel2", config2);
      
      addCommands(
          m_robotDrive.createCommandForTrajectory(trajectory1, true).withTimeout(50).withName("Barrel1"),
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Barrel2")
      );
  }
}