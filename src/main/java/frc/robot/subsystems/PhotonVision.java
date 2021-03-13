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

   public void lightsOn() {
      m_limePhoton.setLED(LEDMode.kOn);
   }

   public void lightsOff() {
      m_limePhoton.setLED(LEDMode.kOff);
   }

   public double getYaw() {
      var result = m_limePhoton.getLatestResult();
      if (result.hasTargets()) {
         return result.getBestTarget().getYaw();
      }
      return -999.0;
   }
}
