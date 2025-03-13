package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraManager 
{
    private PackLog m_Logger;
    
    private PhotonCamera[] m_Cameras;
    private PhotonPoseEstimator[] m_PoseEstimators;

    private AprilTagFieldLayout m_FieldLayout;

    public CameraManager()
    {
        this.m_Logger = new PackLog("CAMERA-MANAGER");
        this.m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    // For completeness this method should be checking for incorrect inputs.
    public boolean InitializeCameras(String[] pipelines, Transform3d[] transforms)
    {
        ArrayList<PhotonCamera> cameraList = new ArrayList<>();
        ArrayList<PhotonPoseEstimator> estimatorList = new ArrayList<>();

        int i = 0;
        for (String pipeline : pipelines) 
        {
            cameraList.add(new PhotonCamera(pipeline));
            estimatorList.add(new PhotonPoseEstimator(m_FieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, transforms[i]));
            i++;
        }

        this.m_Cameras = cameraList.toArray(PhotonCamera[]::new);
        this.m_PoseEstimators = estimatorList.toArray(PhotonPoseEstimator[]::new);

        this.m_Logger.Log("Completed camera initialization.");
        this.m_Logger.Log("----- Current Camera Status -----");

        for (PhotonCamera camera : this.m_Cameras)
        {
            if(camera.isConnected())
            {
                this.m_Logger.Log(String.format("%-15s %15s%n", camera.getName(), "CONNECTED"));
            }
            this.m_Logger.Log(String.format("%-15s %15s%n", camera.getName(), "DISCONNECTED"));
        }

        return true; // always return true for now
    }

    public void UpdateCameras(BiConsumer<Pose2d, Double> addVisionMeasurement)
    {
        int i = 0;
        for (PhotonCamera camera : this.m_Cameras)
        {
            if(!camera.isConnected())
            {
                continue;
            }

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            if(results.isEmpty())
            {
                continue;
            }

            Optional<EstimatedRobotPose> possiblePose = this.m_PoseEstimators[i].update(results.get(0));
            if(possiblePose.isPresent())
            {
                EstimatedRobotPose estimatedPose = possiblePose.get();
                addVisionMeasurement.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
                // addVisionMeasurement.apply(estimatedPose.estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
            }

            i++;
        }
    }
}
