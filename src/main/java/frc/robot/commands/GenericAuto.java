package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class GenericAuto extends SequentialCommandGroup {
  public GenericAuto(DriveSubsystem m_robotDrive, String filename) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile(filename);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(15).withName(filename)
      );
  }
}
