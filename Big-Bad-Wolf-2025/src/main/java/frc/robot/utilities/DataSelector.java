package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataSelector extends SubsystemBase {
    
    ArrayList<Column> columns;
    Column currentColumn;
    int currentColumnIndex;

    public DataSelector() {
        columns = new ArrayList<>();
    }

    public void addColumn(Column newColumn) {
        columns.add(newColumn);

        if(currentColumn == null) {
            currentColumn = newColumn;
            currentColumnIndex = 0;
        }
    }

    public Column getColumn(String columnName) {
        for (Column c : columns) {
            if(c.getColumnName().equals(columnName)) {
                return c;
            }
        }
        // return something better.
        DataLogManager.log("ERROR: invalid column");
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }
        return null;
    }

    public boolean hasColumn(String columnName) {
        for (Column c : columns){
            if(c.getColumnName().equals(columnName)) {
                return true;
            }
        }
        return false;
    }

    public Column getCurrentColumn() {
        return this.currentColumn;
    }

    public int getCurrentColumnIndex() {
        return this.currentColumnIndex;
    }

    public Command shiftColumnCategoryLeft() {
        return run(() -> {
            if (currentColumnIndex - 1 >= 0) {
                currentColumn = columns.get(currentColumnIndex - 1);
                currentColumnIndex--;
            }
        });
    }

    public Command shiftColumnCategoryRight() {
        return run(() -> {
            if (currentColumnIndex + 1 < columns.size()) {
                currentColumn = columns.get(currentColumnIndex + 1);
                currentColumnIndex++;
            }
        });
    }

    public Command toggleUpColumn() {
        return run(() -> {
            currentColumn.decrementColumn();
        });
    }

    public Command toggleDownColumn() {
        return run(() -> {
            currentColumn.incrementColumn();
        });
    }

    public Command selectValue() {
        return run(() -> {
            currentColumn.setOptionStateTrue();
        });
    }

    @Override
    public void periodic() {
        //Intentionally Empty
    }
}
