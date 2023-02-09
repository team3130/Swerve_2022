package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
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
import edu.wpi.first.math.filter.MedianFilter;

import javax.swing.text.Position;
import java.io.IOException;
import java.util.ArrayList;

public class Limelight {

    PhotonCamera camera;
    public final GenericEntry ntHasTarget;
    public final GenericEntry ntDifferentTargets;
    private final GenericEntry ntID;
    private static ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    private Field2d fieldPos = new Field2d();

    private MedianFilter xFilter;
    private MedianFilter yFilter;
    private MedianFilter zFilter;
    private MedianFilter yawFilter;
    private MedianFilter pitchFilter;
    private MedianFilter rollFilter;

    public Limelight() {
        camera = new PhotonCamera("OV5647");
        ntHasTarget = tab.add("HasTarget", false).getEntry();
        ntDifferentTargets = tab.add("DifferentTargets", new Long[0]).getEntry();
        ntID = tab.add("ID", 0).getEntry();

        xFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);
        yFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);
        zFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);
        yawFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);
        pitchFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);
        rollFilter = new MedianFilter(Constants.kLimelightFilterBufferSize);

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(IOException e){
            DriverStation.reportError("error loading field position file", false);
        }
    }

    public void updateFilters() {
        Pose3d pose3d = getCameraPosition();
        xFilter.calculate(pose3d.getX());
        yFilter.calculate(pose3d.getY());
        zFilter.calculate(pose3d.getZ());
        yawFilter.calculate(pose3d.getRotation().getAngle());
    }

    public Pose2d updateFilters2d() {
        Pose3d pose3d = getCameraPosition();
        Double x = xFilter.calculate(pose3d.getX());
        Double y = yFilter.calculate(pose3d.getY());
        Double yaw = yawFilter.calculate(pose3d.getRotation().getAngle());
        return new Pose2d(new Translation2d(x, y), new Rotation2d(yaw));
    }
    
    public void outputToShuffleBoard() {

        PhotonPipelineResult result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        updateFilters();
        Transform3d cameraToCenterOfBot = new Transform3d(
                new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));



        if (target != null) {
            Pose3d PoseOnField = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToCenterOfBot);

            fieldPos.setRobotPose(PoseOnField.toPose2d());
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

        if(!result.hasTargets()){
            return null;
        }
        PhotonTrackedTarget target = result.getBestTarget();
        // x is forward, y is left, z is up
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();

        updateFilters();

        // the matrix transformation for the camera to the center of the bot
        Transform3d cameraToCenterOfBot = new Transform3d(
                new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

        return PhotonUtils.estimateFieldToRobotAprilTag(
                bestCameraToTarget,
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                cameraToCenterOfBot);
    }
}

