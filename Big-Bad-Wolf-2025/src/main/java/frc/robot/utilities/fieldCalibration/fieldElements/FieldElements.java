package frc.robot.utilities.fieldCalibration.fieldElements;

public class FieldElements {

    public static enum FIELD_SIDE {
        RED,
        BLUE
    };   
    
    private FIELD_SIDE fieldSide;
    private String fieldElementName;

    protected FieldElements(FIELD_SIDE side, String name) {
        this.fieldSide = side;
        setFieldElementName(name);
    }

    public void setSide(FIELD_SIDE fieldSide) {
        this.fieldSide = fieldSide;
    }

    public void setFieldElementName(String name) {
        this.fieldElementName = fieldSide + "_" + name;
    }

    public FIELD_SIDE getSide() {
        return this.fieldSide;
    }
    
    public String getFieldElementName() {
        return this.fieldElementName;
    }
}
