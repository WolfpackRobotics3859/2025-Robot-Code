package frc.robot.utilities.dataSelector;

public class DataSelectorHelper
{
    public static final Column HEADER_COLUMN = new Column.ColumnBuilder()
    .addColumnName("headerColumn")
    .addFirstOption("LeftORRight", true)
    .addOption("Levels")
    .addOption("Face")
    .build();


    public static final Column LEFT_RIGHT_CLEAN_COLUMN = new Column.ColumnBuilder()
    .addColumnName("leftRight")
    .addFirstOption("Left", true)
    .addOption("Right")
    .build();

    public static final Column LEVELS_COLUMN = new Column.ColumnBuilder()
    .addColumnName("levels")
    .addFirstOption("one", true)
    .addOption("two")
    .addOption("three")
    .addOption("four")
    .build();

    public static final Column REEF_FACE_SELECTION_COLUMN = new Column.ColumnBuilder()
    .addColumnName("reefFaceSelection")
    .addFirstOption("reef1", true)
    .addOption("reef2")
    .addOption("reef3")
    .addOption("reef4")
    .addOption("reef5")
    .addOption("reef6")
    .build();
}
