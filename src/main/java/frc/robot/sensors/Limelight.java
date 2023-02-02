package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;

public class Limelight {
    PhotonCamera camera;
    public final GenericEntry ntHasTarget;
    public final GenericEntry ntDifferentTargets;
    private final GenericEntry ntID;
    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    public Limelight() {
        camera = new PhotonCamera("OV5647");
        ntHasTarget = tab.add("HasTarget", false).getEntry();
        ntDifferentTargets = tab.add("DifferentTargets", new Long[0]).getEntry();
        ntID = tab.add("ID", 0).getEntry();

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(IOException e){
            DriverStation.reportError("error loading field position file", false);
        }
    }

    public void outputToShuffleBoard() {

        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        if (target != null) {
            boolean hasTargets = result.hasTargets();
            int targetID = target.getFiducialId();

            ntHasTarget.setBoolean(hasTargets);
            ntID.setInteger(target.getFiducialId());

            ArrayList<PhotonTrackedTarget> diffrentID = new ArrayList<>(result.getTargets());
            Long[] fiducialIDs = new Long[diffrentID.size()];

            for (int index = 0; index < diffrentID.size(); index++) {
                fiducialIDs[index] = ((long) diffrentID.get(index).getFiducialId());

            }
        }
    }
    public Pose3d position() {

        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d origin = new Transform3d(new Translation3d(0, 0,0), new Rotation3d(0, 0, 0));

        ArrayList<PhotonTrackedTarget> targetPose = new ArrayList<>(result.getTargets());
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                origin);

        return robotPose;

    }
}
