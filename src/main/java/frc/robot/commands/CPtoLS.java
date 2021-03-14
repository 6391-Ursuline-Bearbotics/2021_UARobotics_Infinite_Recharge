package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CPtoLS extends SequentialCommandGroup {
   public CPtoLS(ShooterSubsystem m_shooter, DriveSubsystem m_robotDrive, IntakeSubsystem m_intake) {        
      Trajectory trajectory1 = m_robotDrive.loadTrajectoryFromFile("CP to LS1");
      Trajectory trajectory2 = m_robotDrive.loadTrajectoryFromFile("CP to LS2");
         
      addCommands(
         // Resets odometery which is what defines where the robot is.  This isn't generally nessecary for a match auto because the
         // robot will start on the field at the right spot (hopefully) but it is useful for testing / simulation.    
         new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),

         // Turn off Shooter
         new InstantCommand(() -> {
         m_shooter.setSetpoint(0);
         m_shooter.disable();
         }, m_shooter),

         // Back up to the wall to get out from under the CP
         m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("CP to LS1"),

         new ParallelCommandGroup(
            // Wait a bit and then deploy the intake for recieving balls from LS
            new WaitCommand(2).andThen(
               new InstantCommand(() -> m_intake.deployIntake())),        

            // Run the trajectory driving us forward to the LS
            m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("CP to LS2"))
      );
   }
}