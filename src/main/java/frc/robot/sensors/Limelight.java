package frc.robot.sensors;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class Limelight {

    PhotonCamera camera;

    private final GenericEntry ntHasTarget;
    private final GenericEntry ntYaw;
    private final GenericEntry ntDifferentTargets;
    private final GenericEntry ntID;


    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");

    public Limelight() {
        camera = new PhotonCamera("OV5647");
        ntHasTarget = tab.add("HasTarget", false).getEntry();
        ntYaw = tab.add("Yaw", 0).getEntry();
        ntDifferentTargets = tab.add("DifferentTargets", new Long[0]).getEntry();
        ntID = tab.add("ID", 0).getEntry();
    }

    public void outputToShuffleBoard() {

        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        if (target != null) {
            boolean hasTargets = result.hasTargets();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getPitch();
            double skew = target.getSkew();
            int targetID = target.getFiducialId();

            ntHasTarget.setBoolean(hasTargets);
            ntYaw.setDouble(target.getYaw());
            ntID.setInteger(target.getFiducialId());


            ArrayList<PhotonTrackedTarget> diffrentID = new ArrayList<>(result.getTargets());
            Long[] fiducialIDs = new Long[diffrentID.size()];

            for (int index = 0; index < diffrentID.size(); index++) {
                fiducialIDs[index] = ((long) diffrentID.get(index).getFiducialId());

            }
        }
    }
}
