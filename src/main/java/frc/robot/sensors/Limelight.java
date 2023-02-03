package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.Camera;

import java.io.IOException;
import java.util.ArrayList;

import static frc.robot.Constants.Camera.xPos;

public class Limelight {
    PhotonCamera camera;
    public final GenericEntry ntHasTarget;
    public final GenericEntry ntDifferentTargets;
    private final GenericEntry ntID;
    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    private Field2d fieldPos = new Field2d();
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
        Transform3d cameraToCenterOfBot = new Transform3d(
                new Translation3d(xPos, Camera.yPos, Camera.zPos),
                new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));
        Pose3d PoseOnField = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToCenterOfBot);

        fieldPos.setRobotPose(PoseOnField.toPose2d());

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
    public Pose3d getCameraPosition() {
        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        // x is forward, y is left, z is up
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();

        // the matrix transformation for the camera to the center of the bot
        Transform3d cameraToCenterOfBot = new Transform3d(
                new Translation3d(xPos, Camera.yPos, Camera.zPos),
                new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

        return PhotonUtils.estimateFieldToRobotAprilTag(
                bestCameraToTarget,
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                cameraToCenterOfBot);
    }



}

