package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.UA6391.Trajectory6391;
import frc.robot.subsystems.DriveSubsystem;

public class Barrel extends SequentialCommandGroup {
  public Barrel(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig config = new TrajectoryConfig(1.8, 3);
      config.setEndVelocity(5);
      TrajectoryConfig config2 = new TrajectoryConfig(3.5, 5);
      config2.setStartVelocity(5);
      
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Barrel1"); //, config);
      Trajectory trajectory2 = m_robotDrive.generateTrajectory("Barrel2", config2);
      var events = m_robotDrive.getEventTimes("Barrel2", trajectory2, Arrays.asList(1));
      
      addCommands(
          m_robotDrive.createCommandForTrajectory(trajectory1, true).withTimeout(50).withName("Barrel1"),
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Barrel2")
      );
  }
}