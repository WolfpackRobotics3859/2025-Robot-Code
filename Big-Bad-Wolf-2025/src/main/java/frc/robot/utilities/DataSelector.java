package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.dataSelector.Column;

public class DataSelector extends SubsystemBase 
{
    private final Column header;
    private ArrayList<Column> columns;
    private Column currentColumn;
    private int currentColumnIndex;

    public DataSelector(Column header)
     {
        this.header = header;
        columns = new ArrayList<>();
    }

    public void addColumn(Column newColumn) {
        columns.add(newColumn);

        if(currentColumn == null) {
            currentColumn = newColumn;
            currentColumnIndex = 0;
        }
    }

    public Column getColumn(int id)
    {
        return this.columns.get(id);
    }

    public boolean hasColumn(String columnName) {
        for (Column c : columns){
            if(c.getName().equals(columnName)) {
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
        return runOnce(() -> {
            if(header.decrementColumn()) {
                currentColumnIndex--;
                currentColumn = columns.get(currentColumnIndex);
            }
        });
    }

    public Command shiftColumnCategoryRight() {
        return runOnce(() -> {
            if (header.incrementColumn()) {
                currentColumnIndex++;
                currentColumn = columns.get(currentColumnIndex);
            }
        });
    }

    public Command toggleUpColumn() {
        return runOnce(() -> {
            currentColumn.decrementColumn();
        });
    }

    public Command toggleDownColumn() {
        return runOnce(() -> {
            currentColumn.incrementColumn();
        });
    }

    public Command dumpData() {
        return runOnce(() -> {
            // SmartDashboard.putString("Current Column Name", this.currentColumn.getName());
            // SmartDashboard.putString("Current option", this.currentColumn.getCurrentOption().getName());
            // SmartDashboard.putBoolean("Current option state", this.currentColumn.getCurrentOption().getState());
            for(Column c : columns) {
                SmartDashboard.putString(c.getName(), c.getCurrentOption().getName());
            }
        });
    }

    @Override
    public void periodic() {
        //Intentionally Empty
    }
}
