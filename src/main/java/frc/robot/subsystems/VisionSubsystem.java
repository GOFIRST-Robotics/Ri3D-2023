package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_ID);
    boolean hasTarget; // Stores whether or not a target is detected
    PhotonPipelineResult result;

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult(); // Query the latest result from PhotonVision
        hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
        if (hasTarget) {
            this.result = result;
        }

    }
    public PhotonTrackedTarget getTargetWithID(int id) { // Checks if the ID we want is the ID
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) {
                return i; // Found the target with the ID
            }
        }
        return null; // Failed to find the target with the ID
            
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
        return result.getBestTarget(); // Returns the best target
        }
        else {
            return null; // Otherwise, returns null
        }
    }
    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }


}
