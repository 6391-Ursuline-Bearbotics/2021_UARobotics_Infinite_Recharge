package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GalacticSearchAuto extends SequentialCommandGroup {
   public GalacticSearchAuto(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake) {
      addCommands(
         // Deploy intake
         new InstantCommand(() -> m_intake.setOutput(IntakeConstants.kIntakeMotorSpeed))
         .andThen(new InstantCommand(() -> m_intake.extendIntake(true))),
         new WaitCommand(0.2), // need to wait for intake to drop so not obstructing the camera
         new GalacticSearch(m_robotDrive, m_intake)
      );
   }
}
