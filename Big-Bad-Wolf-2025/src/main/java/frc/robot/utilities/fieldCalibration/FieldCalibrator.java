package frc.robot.utilities.fieldCalibration;

import java.io.File;
import java.io.FileWriter;
import java.math.BigDecimal;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.fieldCalibration.fieldElements.Barge;
import frc.robot.utilities.fieldCalibration.fieldElements.FeederStation;
import frc.robot.utilities.fieldCalibration.fieldElements.FieldElements;
import frc.robot.utilities.fieldCalibration.fieldElements.ProcessorStation;
import frc.robot.utilities.fieldCalibration.fieldElements.Reef;
import frc.robot.utilities.fieldCalibration.fieldElements.FeederStation.LEFT_OR_RIGHT_FEEDER;

public class FieldCalibrator 
{
    private CommandSwerveDrivetrain commandSwerveDrivetrain;

    @JsonProperty
    private Reef redReef;
    @JsonProperty
    private Reef blueReef;

    @JsonProperty
    private Barge redBarge;
    @JsonProperty
    private Barge blueBarge;

    @JsonProperty
    private FeederStation redLeftFeederStation;
    @JsonProperty
    private FeederStation redRightFeederStation;

    @JsonProperty
    private FeederStation blueLeftFeederStation;
    @JsonProperty
    private FeederStation blueRightFeederStation;

    @JsonProperty
    private ProcessorStation redProcessor;
    @JsonProperty
    private ProcessorStation blueProcessor;

    private List<FieldElements> fieldElements;

    private boolean redSide;
    private boolean blueSide;
    private boolean reefSelected;
    private boolean bargeSelected;
    private boolean feederSelected;
    private boolean processorSelected;

    private SendableChooser <Reef.FACE_VALUE> reefFaceNumber;
    private SendableChooser <Reef.LEFT_RIGHT_CLEAN> reefLRCSelection;
    private SendableChooser <FeederStation.LEFT_OR_RIGHT_FEEDER> leftRightFeeder;
    private SendableChooser <FeederStation.POSITION_OF_FEEDING> positionOfFeeding;
    private SendableChooser <Barge.CAGES_AND_BARGE_SHOT> bargePositionOfCage;

    public FieldCalibrator(CommandSwerveDrivetrain commandSwerveDrivetrain) 
    {
        redReef = new Reef(FieldElements.FIELD_SIDE.RED);
        blueReef = new Reef(FieldElements.FIELD_SIDE.BLUE);
        redBarge = new Barge(FieldElements.FIELD_SIDE.RED);
        blueBarge = new Barge(FieldElements.FIELD_SIDE.BLUE);
        redLeftFeederStation = new FeederStation(FieldElements.FIELD_SIDE.RED, FeederStation.LEFT_OR_RIGHT_FEEDER.LEFT);
        redRightFeederStation = new FeederStation(FieldElements.FIELD_SIDE.RED,FeederStation.LEFT_OR_RIGHT_FEEDER.RIGHT);
        blueLeftFeederStation = new FeederStation(FieldElements.FIELD_SIDE.BLUE, FeederStation.LEFT_OR_RIGHT_FEEDER.LEFT);
        blueRightFeederStation = new FeederStation(FieldElements.FIELD_SIDE.BLUE, FeederStation.LEFT_OR_RIGHT_FEEDER.RIGHT);
        redProcessor = new ProcessorStation(FieldElements.FIELD_SIDE.RED);
        blueProcessor = new ProcessorStation(FieldElements.FIELD_SIDE.BLUE);

        fieldElements = List.of(
            redReef,
            blueReef,
            redBarge,
            blueBarge,
            redLeftFeederStation,
            redRightFeederStation,
            blueLeftFeederStation,
            blueRightFeederStation,
            redProcessor,
            blueProcessor);

        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        redSide = SmartDashboard.putBoolean("Red Side", false);
        blueSide = SmartDashboard.putBoolean("Blue Side", false);
        reefSelected = SmartDashboard.putBoolean("Reef", false);
        bargeSelected = SmartDashboard.putBoolean("Barge", false);
        feederSelected = SmartDashboard.putBoolean("Feeder", false);
        processorSelected = SmartDashboard.putBoolean("Processor", false);

        buildSendables();
    }

    private void buildSendables() {
        reefFaceNumber = new SendableChooser<>();
        reefFaceNumber.setDefaultOption( "Face1", Reef.FACE_VALUE.FACE_ONE);
        reefFaceNumber.addOption( "Face2", Reef.FACE_VALUE.FACE_TWO);
        reefFaceNumber.addOption( "Face3", Reef.FACE_VALUE.FACE_THREE);
        reefFaceNumber.addOption( "Face4", Reef.FACE_VALUE.FACE_FOUR);
        reefFaceNumber.addOption("Face5", Reef.FACE_VALUE.FACE_FIVE);
        reefFaceNumber.addOption("Face6", Reef.FACE_VALUE.FACE_SIX);
        SmartDashboard.putData(reefFaceNumber);

        reefLRCSelection = new SendableChooser<>();
        reefLRCSelection.setDefaultOption("Left", Reef.LEFT_RIGHT_CLEAN.LEFT);
        reefLRCSelection.addOption("Right", Reef.LEFT_RIGHT_CLEAN.RIGHT);
        reefLRCSelection.addOption("Clean", Reef.LEFT_RIGHT_CLEAN.CLEAN);
        SmartDashboard.putData(reefLRCSelection);

        leftRightFeeder = new SendableChooser<>();
        leftRightFeeder.setDefaultOption("LeftFeederStation", FeederStation.LEFT_OR_RIGHT_FEEDER.LEFT);
        leftRightFeeder.addOption("RightFeederStation", FeederStation.LEFT_OR_RIGHT_FEEDER.RIGHT);
        SmartDashboard.putData(leftRightFeeder);

        positionOfFeeding = new SendableChooser<>();
        positionOfFeeding.setDefaultOption("LeftFeeding", FeederStation.POSITION_OF_FEEDING.LEFT_FEED);
        positionOfFeeding.addOption("RightFeeding", FeederStation.POSITION_OF_FEEDING.RIGHT_FEED);
        positionOfFeeding.addOption("CenterFeeding", FeederStation.POSITION_OF_FEEDING.CENTER_FEED);
        SmartDashboard.putData(positionOfFeeding);

        bargePositionOfCage = new SendableChooser<>();
        bargePositionOfCage.setDefaultOption("LeftCage", Barge.CAGES_AND_BARGE_SHOT.LEFT_CAGE);
        bargePositionOfCage.addOption("RightCage", Barge.CAGES_AND_BARGE_SHOT.RIGHT_CAGE);
        bargePositionOfCage.addOption("CenterCage", Barge.CAGES_AND_BARGE_SHOT.CENTER_CAGE);
        bargePositionOfCage.addOption("bargeShot", Barge.CAGES_AND_BARGE_SHOT.BARGE_SHOT);
        SmartDashboard.putData(bargePositionOfCage);
    }

    @JsonIgnore
    public Pose2d getPose2d() 
    {
        return this.commandSwerveDrivetrain.getState().Pose;
    }

    @JsonIgnore
    public List<FieldElements> getFieldElements()
    {
        return fieldElements;
    }

    public void calibratePosition() {
        List<Boolean> fieldElements = getValues();

        if((redSide && blueSide) || (!redSide && !blueSide)) {
            System.out.println("ERROR: both sides selected or neither selected please try again");
            return;
        }
        
        if( !fieldElements.contains(true) || 
        (fieldElements.indexOf(true) != fieldElements.lastIndexOf(true))) 
        {
            System.out.println("ERROR: please select only one field element at a time please try again");
            return;
        }

        if(redSide) {
            chooser(FieldElements.FIELD_SIDE.RED);
        } else {
            chooser(FieldElements.FIELD_SIDE.BLUE);
        }
    }

    private List<Boolean> getValues() {
        redSide = SmartDashboard.getBoolean("Red Side", false);
        blueSide = SmartDashboard.getBoolean("Blue Side", false);
        reefSelected = SmartDashboard.getBoolean("Reef", false);
        bargeSelected = SmartDashboard.getBoolean("Barge", false);
        feederSelected = SmartDashboard.getBoolean("Feeder", false);
        processorSelected = SmartDashboard.getBoolean("Processor", false);
        
        return List.of(reefSelected, bargeSelected, feederSelected, processorSelected);
    }

    private void chooser(FieldElements.FIELD_SIDE fSide) {
        if(fSide.equals(FieldElements.FIELD_SIDE.RED)) {
            if(reefSelected) {
                redReef.addFace(reefFaceNumber.getSelected(), reefLRCSelection.getSelected(), getPose2d());
            } else if (bargeSelected) {
                redBarge.addCageLocations(bargePositionOfCage.getSelected(), getPose2d());
            } else if (feederSelected) {
                if(leftRightFeeder.getSelected() == LEFT_OR_RIGHT_FEEDER.LEFT) {
                    redLeftFeederStation.addFeederLocations(positionOfFeeding.getSelected(), getPose2d());
                } else {
                    redRightFeederStation.addFeederLocations(positionOfFeeding.getSelected(), getPose2d());
                }
            } else if(processorSelected) {
                redProcessor.addLocation(getPose2d());
            } else {
                System.out.println("how you do this.");
            }
        }else{
            if(reefSelected) {
                blueReef.addFace(reefFaceNumber.getSelected(), reefLRCSelection.getSelected(), getPose2d());
            } else if (bargeSelected) {
                blueBarge.addCageLocations(bargePositionOfCage.getSelected(), getPose2d());
            } else if (feederSelected) {
                if(leftRightFeeder.getSelected() == LEFT_OR_RIGHT_FEEDER.LEFT) {
                    blueLeftFeederStation.addFeederLocations(positionOfFeeding.getSelected(), getPose2d());
                } else {
                    blueRightFeederStation.addFeederLocations(positionOfFeeding.getSelected(), getPose2d());
                }
            } else if(processorSelected) {
                blueProcessor.addLocation(getPose2d());
            } else {
                System.out.println("how you do this.");
            }
        }
    }


    public void generateConstantsFile(File outputFile) {

        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append("package frc.robot.generated;\n\n");
        stringBuilder.append("import edu.wpi.first.math.geometry.Pose2d;");
        stringBuilder.append("import edu.wpi.first.math.geometry.Rotation2d;\n\n");
        stringBuilder.append("public class FieldCalibrationConstants {\n\n");
        for(FieldElements fieldElement : fieldElements) {
            if(fieldElement instanceof ProcessorStation) {
                ProcessorStation temp = (ProcessorStation)fieldElement;
                for(String key : temp.getLocation().keySet())
                {
                    try {
                        stringBuilder.append(formatValues(
                            temp.getFieldElementName(), 
                            temp.getLocation().get(key).getX(), 
                            temp.getLocation().get(key).getY(), 
                            temp.getLocation().get(key).getRotation().getRadians()));
                    } catch (NullPointerException e) {
                        System.out.printf("ERROR IN WRITING %s CONSTANTS MIGHT BE MISSING DATA\n", temp.getFieldElementName());
                        stringBuilder.append(formatValues(temp.getFieldElementName()));
                    }
                }
                stringBuilder.append("\n\n");
            } else if(fieldElement instanceof FeederStation) {
                FeederStation tempFeederStation = (FeederStation) fieldElement;
                for(FeederStation.POSITION_OF_FEEDING pOf : tempFeederStation.getFeederLocations().keySet()) {
                    try {
                        stringBuilder.append(formatValues(
                            tempFeederStation.getFieldElementName() +"_"+ pOf, 
                            tempFeederStation.getFeederLocations().get(pOf).getX(), 
                            tempFeederStation.getFeederLocations().get(pOf).getY(), 
                            tempFeederStation.getFeederLocations().get(pOf).getRotation().getRadians()));
                        } catch( NullPointerException e) {
                            System.out.printf("ERROR IN WRITING %s CONSTANTS MIGHT BE MISSING DATA\n", tempFeederStation.getFieldElementName() +"_"+ pOf);
                            stringBuilder.append(formatValues(tempFeederStation.getFieldElementName() +"_"+ pOf));
                        }
                }
                stringBuilder.append("\n\n");
            } else if(fieldElement instanceof Barge) {
                Barge tempBarge = (Barge) fieldElement;
                for(Barge.CAGES_AND_BARGE_SHOT cAb : tempBarge.getBargeLocations().keySet()) {
                    try {
                        stringBuilder.append(formatValues(
                            tempBarge.getFieldElementName() + "_" + cAb,
                            tempBarge.getBargeLocations().get(cAb).getX(), 
                            tempBarge.getBargeLocations().get(cAb).getY(), 
                            tempBarge.getBargeLocations().get(cAb).getRotation().getRadians()));
                    } catch (NullPointerException e) {
                        System.out.printf("ERROR IN WRITING %s CONSTANTS MIGHT BE MISSING DATA\n", tempBarge.getFieldElementName() + "_" + cAb);
                        stringBuilder.append(formatValues(tempBarge.getFieldElementName() + "_" + cAb));
                    }
                }
                stringBuilder.append("\n\n");
            } else {
                Reef tempReef = (Reef) fieldElement;
                for(Reef.FACE_VALUE fv : tempReef.getFaces().keySet()) {
                    for(Reef.LEFT_RIGHT_CLEAN lRc : tempReef.getFaces().get(fv).keySet()) {
                        try {
                            stringBuilder.append(formatValues(
                                tempReef.getFieldElementName() + "_" + fv + "_" + lRc,
                                tempReef.getFaces().get(fv).get(lRc).getX(), 
                                tempReef.getFaces().get(fv).get(lRc).getY(), 
                                tempReef.getFaces().get(fv).get(lRc).getRotation().getRadians()));
                        } catch (NullPointerException e) {
                            System.out.printf("ERROR IN WRITING %s CONSTANTS MIGHT BE MISSING DATA\n", tempReef.getFieldElementName() + "_" + fv + "_" + lRc);
                            stringBuilder.append(formatValues(tempReef.getFieldElementName() + "_" + fv + "_" + lRc));
                        }
                    }
                    stringBuilder.append("\n"); 
                }
                stringBuilder.append("\n\n");
     
            }
        }        
        stringBuilder.append("}");


        try (FileWriter writer = new FileWriter(outputFile)) {
            writer.write(stringBuilder.toString());
            
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    private String formatValues(String name, double xValue, double yValue, double radianValue) {
        return String.format("public static Pose2d %s = new Pose2d(%.4f, %.4f, new Rotation2d(%.4f));\n", name, xValue, yValue, radianValue);
        // return "public static Pose2d " + name + " = new Pose2d(" + BigDecimal.valueOf(xValue) + ", " + BigDecimal.valueOf(yValue) + ", new Rotation2d(" + BigDecimal.valueOf(radianValue) + "));\n";
    }

    private String formatValues(String name) {
        return "public static Pose2d " + name + " = null;\n";
    }

}
