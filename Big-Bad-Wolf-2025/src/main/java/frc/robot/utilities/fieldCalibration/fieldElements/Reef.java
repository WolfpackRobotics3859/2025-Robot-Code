package frc.robot.utilities.fieldCalibration.fieldElements;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;

public class Reef extends FieldElements
{
    public static enum LEFT_RIGHT_CLEAN
    {
        RIGHT,
        CLEAN,
        LEFT
    }

    public static enum FACE_VALUE
    {
        FACE_ONE,
        FACE_TWO,
        FACE_THREE,
        FACE_FOUR,
        FACE_FIVE,
        FACE_SIX
    }

    //<FaceName<Left/Right/Clean,Pose2d>>
    private Map<FACE_VALUE, Map<LEFT_RIGHT_CLEAN, Pose2d>> faces;

    public Reef(FIELD_SIDE fieldSide) 
    {
        super(fieldSide, "REEF");
        faces = new TreeMap<>();
        loadFaces();
    }

    private void loadFaces() {
        for(FACE_VALUE face : FACE_VALUE.values()) {
            faces.put(face, new TreeMap<>()
            {
                {
                    for(LEFT_RIGHT_CLEAN lrc : LEFT_RIGHT_CLEAN.values()) 
                    {
                        put(lrc, null);
                    }
                }
            });
        }    
    }

    public Map<FACE_VALUE, Map<LEFT_RIGHT_CLEAN, Pose2d>> getFaces() {
        return faces;
    }

    public void addFace(FACE_VALUE faceValue, LEFT_RIGHT_CLEAN leftRightClean, Pose2d inputPose) 
    {
        if(faces.get(faceValue).get(leftRightClean) != null) 
        {
            System.err.printf("WARNING: %s_REEF_%s_%s DATA ALREADY EXISTS OVERRIDNG CURRENT DATA\n", super.getSide(), faceValue, leftRightClean);
            faces.get(faceValue).remove(leftRightClean);
        } else 
        {
            System.out.printf("%s_REEF_%s_%s: Data added\n", super.getSide(), faceValue, leftRightClean);
        }

        faces.get(faceValue).put(leftRightClean, inputPose); 
    }
}
