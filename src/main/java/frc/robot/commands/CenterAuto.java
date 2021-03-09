package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CenterAuto extends SequentialCommandGroup {
  public CenterAuto(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Center1");
      Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Center2");
      Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Center3");
      
      addCommands(
          new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),

          //turn shooter on to get up to speed
          new InstantCommand(() -> m_shooter.setSetpoint(AutoConstants.kAutoShootRPS)),

          //back up 1 meter
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Center1"),

          // shoot 3 balls
          new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShoot3),

          //back up into rendevouz
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Center2"),

          //lower and spin intake
          new InstantCommand(() -> m_intake.deployIntake()),

          //separate Center3 into two paths to raise intake before making sharp turn by pillar

          m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Center3"),

          //shoot other balls
          new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShoot3)
      );
  }
}