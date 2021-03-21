package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonPipelineResult;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;

public class GalacticSearch extends SequentialCommandGroup {
   Supplier<Object> i = () -> "";

   public GalacticSearch(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, PhotonVision m_PhotonVision) {        
      Trajectory trajectory1R = m_robotDrive.loadTrajectoryFromFile("GalacticSearch1R");
      Trajectory trajectory1B = m_robotDrive.loadTrajectoryFromFile("GalacticSearch1B");
      Trajectory trajectory2R = m_robotDrive.loadTrajectoryFromFile("GalacticSearch2R");
      Trajectory trajectory2B = m_robotDrive.loadTrajectoryFromFile("GalacticSearch2B");

      addCommands(     
         new InstantCommand(() -> {
               // Set up camera & get PhotonVision result
               String selectedPath = "";
               final String path;
               m_PhotonVision.m_HD3000.takeOutputSnapshot();
               PhotonPipelineResult result = m_PhotonVision.m_HD3000.getLatestResult();
      
               // Make sure there are targets (if not something is really wrong)
               if (result.hasTargets()) {
                  // Check if there is 2 (Path A) or 3 (Path B) balls in the picture
                  if (result.getTargets().size() != 3) {
                     // Check if the closest ball is really close (Red) or not (Blue)
                     if (result.getBestTarget().getArea() > AutoConstants.kBallArea) {
                        // Run path 1R
                        selectedPath = "1R";
                     }
                     else {
                        // Run path 1B
                        selectedPath = "1B";
                     }
                  }
                  else {
                     // Check if the closest ball is really close (Red) or not (Blue)
                     if (result.getBestTarget().getArea() > AutoConstants.kBallArea) {
                        // Run path 2R
                        selectedPath = "2R";
                     }
                     else {
                        // Run path 2B
                        selectedPath = "2B";
                     }
                  }
               }
      
               if (RobotBase.isSimulation()) { // If our robot is simulated
                  path = "1R";
               }
               else {
                  path = selectedPath;
               }
               SmartDashboard.putString("GalacticSearch", path);
               i  = ()-> path;
         }, m_robotDrive),
      
         // Select Command based on selectedPath from above
         new InstantCommand(() -> new SelectCommand(Map.ofEntries(
            Map.entry("1R", 
               // Reset robot pose to the beginning of 1R and Run it
               m_robotDrive.createCommandForTrajectory(trajectory1R, true).withTimeout(50).withName("GalacticSearch1R")),
            Map.entry("1B", 
               // Reset robot pose to the beginning of 1B and Run it
               m_robotDrive.createCommandForTrajectory(trajectory1B, true).withTimeout(50).withName("GalacticSearch1B")),
            Map.entry("2R", 
               // Reset robot pose to the beginning of 2R and Run it
               m_robotDrive.createCommandForTrajectory(trajectory2R, true).withTimeout(50).withName("GalacticSearch2R")),
            Map.entry("2B", 
               // Reset robot pose to the beginning of 2B and Run it
               m_robotDrive.createCommandForTrajectory(trajectory2B, true).withTimeout(50).withName("GalacticSearch2B"))),
         i).schedule())
      );
   }
}