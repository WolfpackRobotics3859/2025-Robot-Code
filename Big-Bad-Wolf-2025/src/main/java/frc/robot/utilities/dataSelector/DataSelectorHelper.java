package frc.robot.utilities.dataSelector;

public class DataSelectorHelper
{
    public static final Column HEADER_COLUMN = new Column.ColumnBuilder()
    .addColumnName("headerColumn")
    .addFirstOption("LeftORRight", true)
    .addOption("Levels")
    .addOption("Face")
    .addOption("Climb")
    .build();


    public static final Column LEFT_RIGHT_CLEAN_COLUMN = new Column.ColumnBuilder()
    .addColumnName("leftRight")
    .addFirstOption("Coral Left", true)
    .addOption("Coral Right")
    .build();

    public static final Column LEVELS_COLUMN = new Column.ColumnBuilder()
    .addColumnName("levels")
    .addFirstOption("lvl one", true)
    .addOption("lvl two")
    .addOption("lvl three")
    .addOption("lvl four")
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

    public static final Column CLIMB_COLUMN = new Column.ColumnBuilder()
    .addColumnName("climbSelection")
    .addFirstOption("climb", true)
    .addOption("unwind")
    .addOption("stilll")
    .build();
}
