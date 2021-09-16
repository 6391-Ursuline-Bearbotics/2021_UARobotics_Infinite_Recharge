package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;

public class TrenchAuto extends SequentialCommandGroup {
    public TrenchAuto(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, PhotonVision m_PhotonVision) {        
        // Loads all of the trajectories we will need.  This happens on init so we get it out of the way before actually running.
        Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Center1");
        Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Trench2");
        Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Trench3Slow");
      
        addCommands(
            // Resets odometery which is what defines where the robot is.  This isn't generally nessecary for a match auto because the
            // robot will start on the field at the right spot (hopefully) but it is useful for testing / simulation.
            new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),
            
            new InstantCommand(() -> {m_shooter.setSetpoint(AutoConstants.kAutoShootRPS);
                m_shooter.enable();}, m_shooter),

            m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Center1"),

            new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShootInit),

            new InstantCommand(() -> m_intake.deployIntake()),

            m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Trench2"),

/*             new ParallelRaceGroup(
                // Pick up 3 balls in a straight line then turn back towards the goal
                m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(5).withName("Trench3Slow"),

                //turn on conveyor
                new RunCommand(() -> {
                        if (m_conveyor.getTopConveyor()) {
                            m_conveyor.turnOn();
                        }
                        else {
                            m_conveyor.turnOff();
                        }
                    }
                , m_conveyor)
            ), */

            new InstantCommand(() -> m_conveyor.turnOn()),

            m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Trench3Slow"),

            new AutoAim(m_robotDrive, m_PhotonVision, m_shooter, () -> 0).withTimeout(.35),

            new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShootRest)
      );

  }
}