package frc.robot.utilities;

public class DataSelectorHelper {
    public static  final Column LEFT_RIGHT_CLEAN_COLUMN = new Column.ColumnBuilder()
    .addColumnName("leftRightClean")
    .addOption("Left")
    .addOption("Right")
    .addOption("Clean")
    .build();

    public static final Column LEVELS_COLUMN = new Column.ColumnBuilder()
    .addColumnName("levels")
    .addOption("one")
    .addOption("two")
    .addOption("three")
    .addOption("four")
    .build();

    public static final Column REEF_FACE_SELECTION_COLUMN = new Column.ColumnBuilder()
    .addColumnName("reefFaceSelection")
    .addOption("one")
    .addOption("two")
    .addOption("three")
    .addOption("four")
    .addOption("five")
    .addOption("six")
    .build();
}
