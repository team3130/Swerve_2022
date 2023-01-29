package frc.robot.sensors;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

public class Limelight {

    PhotonCamera camera;

    private final GenericEntry ntHasTarget;

    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
        public Limelight(){
            camera = new PhotonCamera("OV5647");
            ntHasTarget = tab.add("HasTarget", false).getEntry();
    }

         public void outputToShuffleBoard(){

             PhotonPipelineResult result = camera.getLatestResult();
             PhotonTrackedTarget target = result.getBestTarget();
             boolean hasTargets = result.hasTargets();
             double yaw = target.getYaw();
             double pitch = target.getPitch();
             double area = target.getPitch();
             double skew = target.getSkew();
             int targetID = target.getFiducialId();


             ntHasTarget.setBoolean(hasTargets);
         }
}
