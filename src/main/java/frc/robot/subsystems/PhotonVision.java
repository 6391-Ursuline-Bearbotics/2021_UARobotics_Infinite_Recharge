package frc.robot.subsystems;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PhotonConstants;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limePhoton = new PhotonCamera("gloworm");
   public PhotonCamera m_HD3000 = new PhotonCamera("HD3000");
   public SimVisionSystem visionSys;

   public PhotonVision() {
      m_limePhoton.setPipelineIndex(PhotonConstants.kLimePipe);
      m_HD3000.setPipelineIndex(PhotonConstants.kHD3000Pipe);

      String camName = "gloworm";
      double camDiagFOV = 75.0; // degrees
      double camPitch = LimelightConstants.kCameraAngle;     // degrees
      Transform2d cameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      double camHeightOffGround = LimelightConstants.kCameraHeight; // meters
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
      double targetWidth = 0.8;           // meters
      double targetHeight = 0.762;          // meters
      double targetHeightAboveGround = LimelightConstants.kTargetHeight - targetHeight / 2.0; // meters
      
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
