package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;

public class GalacticSearchAuto extends SequentialCommandGroup {
   public GalacticSearchAuto(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, PhotonVision m_PhotonVision) {
      addCommands(
         // Deploy intake
         new InstantCommand(() -> m_intake.setOutput(IntakeConstants.kIntakeMotorSpeed))
               .andThen(new InstantCommand(() -> m_intake.extendIntake(true))),
         new GalacticSearch(m_robotDrive, m_intake, m_PhotonVision)
      );
   }
}
