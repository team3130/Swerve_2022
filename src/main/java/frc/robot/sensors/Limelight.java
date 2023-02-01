package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import javax.swing.text.Position;
import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class Limelight {

    PhotonCamera camera;
    public final GenericEntry ntHasTarget;
    private final GenericEntry ntYaw;
    public final GenericEntry ntDifferentTargets;
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
    public int robotPose;
    public int targetPose;
    public int cameraToRobot;
    public void Pose2D() {
        
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();

        ArrayList<PhotonTrackedTarget> targetPose = new ArrayList<>(result.getTargets());
            KCameraHeight = targetPose.get()
        }

    }
