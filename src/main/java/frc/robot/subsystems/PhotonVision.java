package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.PhotonConstants;

public class PhotonVision extends SubsystemBase implements Loggable {
   // Creates a new PhotonCamera.
   PhotonCamera m_limePhoton = new PhotonCamera("Limelight");
   PhotonCamera m_HD3000 = new PhotonCamera("HD3000");

   public PhotonVision() {
      m_limePhoton.setPipelineIndex(PhotonConstants.kLimePipe);
      m_HD3000.setPipelineIndex(PhotonConstants.kHD3000Pipe);
   }

   @Log
   public int numberOfTargets(PhotonCamera camera) {
      // Get the latest pipeline result.
      PhotonPipelineResult result = camera.getLatestResult();

      // Get a list of currently tracked targets.
      List<PhotonTrackedTarget> targets = result.getTargets();
      return targets.size();
   }

   @Log
   public boolean hasTarget(PhotonCamera camera){
      return camera.hasTargets();
   }
   
   @Log
   public double getHorizontalError(PhotonCamera camera){
      // Get the latest pipeline result.
      PhotonPipelineResult result = camera.getLatestResult();

      return result.targets.get(0).getYaw();
   }
}
