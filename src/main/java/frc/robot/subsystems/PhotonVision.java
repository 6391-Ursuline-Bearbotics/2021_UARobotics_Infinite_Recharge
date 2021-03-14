package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.PhotonConstants;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limePhoton = new PhotonCamera("Limelight");
   public PhotonCamera m_HD3000 = new PhotonCamera("HD3000");
   public SimVisionSystem visionSys;

   public PhotonVision() {
      m_limePhoton.setPipelineIndex(PhotonConstants.kLimePipe);
      m_HD3000.setPipelineIndex(PhotonConstants.kHD3000Pipe);

      String camName = "Limelight";
      double camDiagFOV = 75.0; // degrees
      double camPitch = 10.0;     // degrees
      Transform2d cameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      double camHeightOffGround = 0.85; // meters
      double maxLEDRange = 20;          // meters
      int camResolutionWidth = 640;     // pixels
      int camResolutionHeight = 480;    // pixels
      double minTargetArea = 10;        // square pixels

      visionSys = new SimVisionSystem(camName,
                                    camDiagFOV,
                                    camPitch,
                                    cameraToRobot,
                                    camHeightOffGround,
                                    maxLEDRange,
                                    camResolutionWidth,
                                    camResolutionHeight,
                                    minTargetArea);

      var targetPose = new Pose2d(new Translation2d(15.97, 2.47), new Rotation2d()); // meters
      double targetHeightAboveGround = 2.3; // meters
      double targetWidth = 0.54;           // meters
      double targetHeight = 0.25;          // meters
      
      var newTgt = new SimVisionTarget(targetPose,
                                       targetHeightAboveGround,
                                       targetWidth,
                                       targetHeight);
      
      visionSys.addSimVisionTarget(newTgt);
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
