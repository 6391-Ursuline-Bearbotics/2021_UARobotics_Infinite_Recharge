package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Barrel extends SequentialCommandGroup {
  public Barrel(DriveSubsystem m_robotDrive) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Barrel1");
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1).withTimeout(15).withName("Barrel1")
      );
  }
}