package frc.robot.utilities.fieldCalibration.fieldElements;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;

public class ProcessorStation extends FieldElements {
    
    private Map<String, Pose2d> location;
    public ProcessorStation(FieldElements.FIELD_SIDE fieldSide) {
        super(fieldSide, "PROCESSOR");
        location = new HashMap<>();
        location.put("PROCESSOR", null);
    }

    public void addLocation(Pose2d pose) 
    {
        if(location.get("PROCESSOR") != null) {
            System.out.printf("WARNING: %s_PROCESSOR_STATION: DATA ALREADY EXISTS OVERRIDNG CURRENT DATA\n", super.getSide());
        }
        else {
            System.out.printf("%s_PROCESSOR_STATION: Data added\n", super.getSide());
        }
        location.replace("PROCESSOR", pose);
    }

    public Map<String, Pose2d> getLocation() 
    {
        return location;
    }

}
