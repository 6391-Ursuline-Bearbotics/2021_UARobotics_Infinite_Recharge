package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;

import frc.robot.Constants.PhotonConstants;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limePhoton = new PhotonCamera("Limelight");
   public PhotonCamera m_HD3000 = new PhotonCamera("HD3000");

   public PhotonVision() {
      m_limePhoton.setPipelineIndex(PhotonConstants.kLimePipe);
      m_HD3000.setPipelineIndex(PhotonConstants.kHD3000Pipe);
   }

   public void beforeTurnToTarget() {
      m_limePhoton.setLED(LEDMode.kOn);
   }

   public void afterTurnToTarget() {
      m_limePhoton.setLED(LEDMode.kOff);
   }

   public void turnToTarget(DriveSubsystem m_robotDrive, DoubleSupplier joystickY) {
      var result = m_limePhoton.getLatestResult();
      if (result.hasTargets()) {
         // Use the joystick for the forward speed and the Yaw as angle
         m_robotDrive.driveToAngle(joystickY, -result.getBestTarget().getYaw()).schedule();
      } else {
         // If we have no targets, stay still.
         m_robotDrive.stopmotors();
      }
   }
}
