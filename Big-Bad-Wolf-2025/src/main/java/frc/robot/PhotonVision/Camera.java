package frc.robot.PhotonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** 
 * A camera object containing PhotonVision objects and camera placement info
 */
public class Camera 
{
    // Photon Objects
    private PhotonCamera PhotonCamera; // The actual camera object
    private PhotonPoseEstimator CameraPoseEstimator; // Estimates the position and orientation of the robot based on vision

    private Transform3d position_3d; // Holds the 3D coordinates of the camera relative to the robot position
    private CAMERA_PLACEMENT placement = CAMERA_PLACEMENT.NONE; // The placement of the camera on the robot

    private int exceptionCount = 0; // Counts the errors of the camera
    private String name; // The name of the camera on the PhotonVision UI

    
    /** Constructs the camera with the appropriate name and placement on the robot.
     * 
     * @param initName The name of the camera on the PhotonVision UI
     * @param initPlacement The placement of the camera on the robot
     * 
     */
    public Camera(String initName, CAMERA_PLACEMENT initPlacement) 
    {
        setCameraName(initName);
        setCameraPlacement(initPlacement);

        setPhotonCamera(new PhotonCamera(this.name));
        setCameraPoseEstimator(new PhotonPoseEstimator(AprilTagInfo.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.position_3d));
        setCameraExceptionCount(0);
        
        if (this.placement == CAMERA_PLACEMENT.FRONT) 
        {
            this.PhotonCamera.setDriverMode(true);
        }
    }

    // GETTER METHODS //

    /** Gets the PhotonCamera object.
     * 
     * @return the PhotonCamera object.
     */
    public final PhotonCamera getPhotonCamera() 
    {
        return this.PhotonCamera;
    }

    /** Gets the PhotonPoseEstimator object.
     * 
     * @return the PhotonPoseEstimator object.
     */
    public final PhotonPoseEstimator getCameraPoseEstimator() 
    {
        return this.CameraPoseEstimator;
    }

    /** Gets the CAMERA_PLACEMENT object which corresponds 
     * to the placement of the camera on the robot.
     * 
     * @return the CAMERA_PLACEMENT object.
     */
    public final CAMERA_PLACEMENT getCameraPlacement() 
    {
        return this.placement;
    }

    /** Gets the Transform3d object which corresponds to the 
     * camera's 3D position relative to the robot.
     * 
     * @return the Transform3d object.
     */
    public final Transform3d getCamera3DPosition() 
    {
        return this.position_3d;
    }

    /** Gets the number of exceptions the camera has encountered.
     * 
     * @return the number of exceptions the camera has encountered.
     */
    public final int getCameraExceptionCount() 
    {
        return this.exceptionCount;
    }

    /** Gets the name of the camera.
     * 
     * @return The name of the camera on the PhotonVision UI.
     */
    public final String getCameraName() 
    {
        return this.name;
    }

    /*
     * SETTER METHODS
     * 
     * With exception of CAMERA_3D_POSITION position_3d,
     * since that is handled when doing setCameraPlacement.
     * 
     */

    public final void setPhotonCamera(PhotonCamera newCamera) 
    {
        this.PhotonCamera = newCamera;
    }

    public final void setCameraPoseEstimator(PhotonPoseEstimator newPoseEstimator) 
    {
        this.CameraPoseEstimator = newPoseEstimator;
    }

    /** Uses the camera's placement to determine its 3D position relative to the robot.
     * 
     * @param newPlacement The placement of the camera on the robot.
     */
    public final void setCameraPlacement(CAMERA_PLACEMENT newPlacement) 
    {
        this.placement = newPlacement;
        switch (placement) 
        {
            case FRONT:
                position_3d = CAMERA_3D_POSITION.FRONT; break;
            case BACK:
                position_3d = CAMERA_3D_POSITION.BACK; break;
            case LEFT:
                position_3d = CAMERA_3D_POSITION.LEFT; break;
            case RIGHT:
                position_3d = CAMERA_3D_POSITION.RIGHT; break;
            case NONE:
                System.out.println("Camera Error: Can't set the placement of a nonexistent camera.");
        }
    }

    /** Sets the camera's exception count to a new integer.
     * 
     * @param newExceptionCount The new exception count of the camera object.
     */
    public final void setCameraExceptionCount(int newExceptionCount) 
    {
        this.exceptionCount = newExceptionCount;
    }

    /** Sets the camera's name to a new String.
     * Must correspond to the camera's nickname on the PhotonVisionUI!
     * 
     * @param newCameraName The new camera name.
     */
    public final void setCameraName(String newName) 
    {
        this.name = newName;
    }


    // TODO: When the cameras are installed, fill in these Translation3d and Rotation3d values to get accurate robot positions on the field
    /** 
     * A class containing the 3D positions for the different placements of the camera.
     */
    public static final class CAMERA_3D_POSITION 
    {

        public static final Transform3d FRONT = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians()
            )
        );

        public static final Transform3d BACK  = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians()
            )
        );

        public static final Transform3d LEFT  = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians()
            )
        );

        public static final Transform3d RIGHT  = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)
            ),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians()
            )
        );
    }

    /**
     * Enumerations which correspond to the different camera (positions) on the robot.
     */
    public static enum CAMERA_PLACEMENT 
    {
        FRONT,
        BACK,
        LEFT,
        RIGHT,
        NONE
    }
}