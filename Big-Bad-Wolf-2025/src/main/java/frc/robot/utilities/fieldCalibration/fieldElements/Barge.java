package frc.robot.utilities.fieldCalibration.fieldElements;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;

public class Barge extends FieldElements{

    public enum CAGES_AND_BARGE_SHOT {
        LEFT_CAGE,
        CENTER_CAGE,
        RIGHT_CAGE,
        BARGE_SHOT
    }

    Map<CAGES_AND_BARGE_SHOT, Pose2d> bargeLocations;    
    
    public Barge(FIELD_SIDE fieldSide) {
        super(fieldSide, "BARGE");
        bargeLocations = new HashMap<>();
    }

    public void addCageLocations(CAGES_AND_BARGE_SHOT cageLocation, Pose2d pose2d) {
        if(bargeLocations.containsKey(cageLocation)) {
            System.err.printf("WARNING: %s_BARGE_%s: DATA ALREADY EXISTS OVERRIDNG CURRENT DATA\n",super.getSide(), cageLocation);
            bargeLocations.remove(cageLocation);
        } else {
            System.out.printf("%s_BARGE_%s: Data added\n",super.getSide(), cageLocation);
        }
        bargeLocations.put(cageLocation, pose2d);
    }

    public Map<CAGES_AND_BARGE_SHOT, Pose2d> getBargeLocations() {
        return bargeLocations;
    }


}
