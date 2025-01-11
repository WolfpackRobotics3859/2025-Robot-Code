package frc.robot.PhotonVision;

import java.io.UncheckedIOException;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** <ul>
 * <li>Uses an AprilTag ID to get its color and area of the field.</li>
 * <li>Contains a PhotonTrackedTarget object for positioning/alignment code.</li>
 * </ul>
 */
public class AprilTagInfo 
{
    AllianceColor color = AllianceColor.NONE;
    Area area = Area.NONE;
    int id = -1;
    PhotonTrackedTarget target;

    
    /** Initializes using an ID to figure out the color/area of the AprilTag.  Also initializes
     * a PhotonTrackedTarget object to use the position of the tag on the camera.
     * 
     * @param initID
     * @param initTarget
     */
    public AprilTagInfo(int initID, PhotonTrackedTarget initTarget) 
    {
        setID(initID);
        setTarget(initTarget);
    }

    /** Initializes using an ID to figure out color/area of the AprilTag.
     * 
     * @param initID
     * @param initTarget
     */
    public AprilTagInfo(int initID) 
    {
        setID(initID);
    }

    //  GETTER METHODS //

    /** Gets the ID of the AprilTag.
     * 
     * @return the ID of the AprilTag.
     */
    public final int getID() 
    {
        return this.id;
    }

    /** Gets the color of the AprilTag according to the FRC manual.
     * 
     * @return the AllianceColor of the AprilTag.
     * @see <a href = "https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">FRC 2025 Game Manual</a>
     */
    public final AllianceColor getColor() 
    {
        return this.color;
    }
    
    /** Gets the area of the AprilTag according to the FRC manual.
     * <ul>
     * <li></li>
     * </ul>
     * 
     * @return the Area of the AprilTag.
     * @see <a href = "https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf">FRC 2025 Game Manual</a>
     */
    public final Area getArea() 
    {
        return this.area;
    }

    /** Gets the PhotonTrackedTarget of the AprilTag which
     * contains information about yaw, pitch, etc. 
     * 
     * @return the PhotonTrackedTarget of the AprilTag.
     */
    public final PhotonTrackedTarget getTarget() 
    {
        return this.target;
    }

    // SETTER METHODS //

    /** Uses a new ID to set the new color and area of the object.
     * 
     * @param newId The new ID to use for setting the areas and color.
     */
    public final void setID(int newId) 
    {
        this.id = newId;
        setArea();
        setColor();
    }

    /** 
     * Sets the color using the current ID of the AprilTagInfo object.
     */
    public final void setColor() 
    {
        switch (this.id) {
            // BLUE
            case 12:
            case 13:
            case 3:
            case 4:
            case 14:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
            this.color = AllianceColor.BLUE;
            break;

            //RED
            case 1:
            case 2:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 15:
            case 16:
            this.color = AllianceColor.RED;
            break;
        }
    }

    /** 
     * Sets the area using the current ID of the AprilTagInfo object.
     */
    public final void setArea() 
    {
        switch (this.id) {
            case 1 : case 2 : case 12 : case 13 :
            this.area = Area.CORAL_STATION;
            break;

            case 3 : case 16 :
            this.area = Area.ALGAE_PROCESSOR;
            break;

            case 4 : case 5 : case 14 : case 15 :
            this.area = Area.CLIMB;
            break;

            case 6 : case 7 : case 8 : case 9 : case 10 : case 11 : // red side
            case 17 : case 18 : case 19 : case 20 : case 21 : case 22 : // blue side
            this.area = Area.CORAL_REEF;
            break;

            default :
            this.area = Area.NONE;
            break;
        }
    }

    /** Sets the PhotonTrackedTarget of the AprilTagInfo object.
     * 
     * @param newTarget The new PhotonTrackedTarget to assign to the object 
     *                  for information about yaw, pitch, etc.
     */
    public final void setTarget(PhotonTrackedTarget newTarget) 
    {
        this.target = newTarget;
    }

    /** 
     * An enumeration corresponding to a side of the field.
     */
    public enum AllianceColor 
    {
        BLUE,
        RED,
        NONE
    }
    
    /**
     * An enumeration corresponding to a region of the field.
     */
    public enum Area 
    {
        CORAL_REEF,
        CORAL_STATION,
        ALGAE_PROCESSOR,
        CLIMB,
        NONE
    }   
    
    public static final AprilTagFieldLayout TAG_LAYOUT;
    
    // Gets the most recent april tag field
    static {
        try {
            TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } 
        catch (UncheckedIOException e) 
        {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }
}
