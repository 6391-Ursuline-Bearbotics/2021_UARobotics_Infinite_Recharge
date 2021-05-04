package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;

public class Center5Ball extends SequentialCommandGroup {
    public Center5Ball(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, PhotonVision m_PhotonVision) {        
        Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("Center1");
        Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("Center2");
        Trajectory trajectory3 = m_robotDrive.loadTrajectoryFromFile("Ball3Slow");
        
        addCommands(
            new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),

            //turn shooter on to get up to speed
            new InstantCommand(() -> {m_shooter.setSetpoint(ShooterConstants.kShooter4);
                m_shooter.enable();}, m_shooter),

            //back up 1 meter
            m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Center1"),

            // shoot 3 balls
            new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShootInit),

            //back up into rendezvous
            m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Center2"),

            //lower and spin intake
            new InstantCommand(() -> m_intake.deployIntake()),

            new ParallelRaceGroup(
                // Pick up 5 balls in a fishook shape and turn back towards the goal
                m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Ball3Slow"),

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
            ),

            // Raise the intake to avoid hitting the pillar
            new InstantCommand(() -> m_intake.retractIntake()),            
            
            // AutoAim to make sure we are pointed directly at the target
            new AutoAim(m_robotDrive, m_PhotonVision, m_shooter, () -> 0),

            //shoot other balls
            new AutoShoot(m_shooter, m_conveyor, AutoConstants.kAutoShootRest)
        );
    }
}
