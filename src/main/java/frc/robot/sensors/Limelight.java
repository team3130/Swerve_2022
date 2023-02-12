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
import frc.robot.supportingClasses.KugelMedianFilter;
import frc.robot.supportingClasses.OdoPosition;
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
    KugelMedianFilter filter;
    int successfulUpdates = 0;

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

        filter = new KugelMedianFilter(Camera.kMedianFilterWindowSize);
    }

    public OdoPosition calculateCameraPosition() {
        PhotonPipelineResult result = camera.getLatestResult();

        if(!result.hasTargets()){
            return null;
        }
        successfulUpdates++;

        PhotonTrackedTarget target = result.getBestTarget();
        // x is forward, y is left, z is up
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();


        // the matrix transformation for the camera to the center of the bot
        Transform3d cameraToCenterOfBot = new Transform3d(
                new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

        Pose3d position = PhotonUtils.estimateFieldToRobotAprilTag(
                bestCameraToTarget,
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                cameraToCenterOfBot);

        return filter.getOdoPose(new OdoPosition(position.toPose2d(), result.getTimestampSeconds()));
    }

    public int getNumberOfSuccesses() {
        return successfulUpdates;
    }

}

