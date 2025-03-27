package frc.robot.utilities.fieldCalibration.fieldElements;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;

public class FeederStation extends FieldElements {

    public static enum POSITION_OF_FEEDING {
        LEFT_FEED,
        CENTER_FEED,
        RIGHT_FEED
    }

    public static enum LEFT_OR_RIGHT_FEEDER {
        LEFT,
        RIGHT
    }

    private LEFT_OR_RIGHT_FEEDER lOr;
    private Map<POSITION_OF_FEEDING, Pose2d> feederLocations;

    public FeederStation(FIELD_SIDE fieldSide, LEFT_OR_RIGHT_FEEDER lOr) {
        super(fieldSide, lOr.toString().concat("_FEEDER_STATION"));
        this.lOr = lOr;
        feederLocations = new HashMap<>();
        preloadFeederStationMap();
    }

    private void preloadFeederStationMap() {
     
        for(POSITION_OF_FEEDING pof : POSITION_OF_FEEDING.values()) {
            feederLocations.put(pof, null);
        }
    }

    public void addFeederLocations(POSITION_OF_FEEDING pOf, Pose2d inputPose) {
        if(feederLocations.get(pOf) != null) 
        {
            System.err.printf("WARNING: %s: DATA ALREADY EXISTS OVERRIDNG CURRENT DATA\n", super.getFieldElementName());
            feederLocations.remove(pOf);
        } else 
        {
            System.out.printf("%s: Data added\n", super.getFieldElementName());
        }

        feederLocations.put(pOf, inputPose);
    }

    public Map<POSITION_OF_FEEDING, Pose2d> getFeederLocations() {
        return feederLocations;
    }

    public LEFT_OR_RIGHT_FEEDER getLeftOrRightFeeder() {
        return lOr;
    }
}
