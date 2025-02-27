package frc.robot.utilities.dataSelector;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Column 
{    
    private String columnName;
    private int chooserCurrentOptionIndex;
    private Option chooserCurrentOption;
    private int currentOptionIndex;
    private Option currentOption;

    
    List<Option> options;

    public Column(ColumnBuilder columnBuilder) {
        this.columnName = columnBuilder.columnName;
        this.options = List.copyOf(columnBuilder.options);
    }

    public int getCurrentOptionIndex() {
        return this.currentOptionIndex;
    }

    public int getChoooserCurrentOptionIndex() {
        return this.chooserCurrentOptionIndex;
    }

    public String getColumnName() {
        return this.columnName;
    }

    public Option getCurrentOption() {
        return this.currentOption;
    }

    public Option getChooserCurrentOption() {
        return this.chooserCurrentOption;
    }

    public void incrementColumn() {
        if(chooserCurrentOptionIndex >= options.size() - 1)
        {
            chooserCurrentOptionIndex ++;
        }
    }

    public void decrementColumn() 
    {
        if(chooserCurrentOptionIndex > 0) 
        {
            chooserCurrentOptionIndex --;
        }
    }

    public void setOptionStateTrue() {
        if (chooserCurrentOption.equals(currentOption)) {
            if (!(currentOption.getState() == true)){
                currentOption.setState(true);
            }
        } else {
            currentOption.setState(false);
            currentOptionIndex = chooserCurrentOptionIndex;
            currentOption = chooserCurrentOption;
            currentOption.setState(true);
        }
    }

    public List<Option> getOptions() {
        return options;
    }

    public void addOption(String name) {
        options.add(new Option(name)); 

        if(currentOption == null) {
            currentOption = options.get(0);
            currentOption.setState(true);
            currentOptionIndex = 0;
            chooserCurrentOption = options.get(0);
            chooserCurrentOptionIndex = 0;
        }
    }

    public Option getOption(String name) {
        for(Option o : options) {
            if(o.getName().equals(name)) {
                return o;
            }
        } 
        return null;
    }

    public boolean hasOption(String name) {
        for(Option o : options) {
            if(o.getName().equals(name)) {
                return true;
            }
        } 
        return false;
    }

    public static class ColumnBuilder 
    {
        private String columnName;
        private ArrayList<Option> options = new ArrayList<>();

        public ColumnBuilder addColumnName(String name) {
            this.columnName = name;
            return this;
        }

        public ColumnBuilder addOption(String optionName) {
            this.options.add(new Option(optionName));
            return this;
        }

        public Column build() {
            return new Column(this);
        }
    }
    
    public static class Option {
        private final String name;
        private boolean state = false;

        public Option(String name) 
        {
            this.name = name;
            SmartDashboard.putBoolean(name, false);
        }

        public void setState(boolean state) {
            this.state = state;
            //FIXME: UPDATE SMART DASHBOARD VALUE

            // update it later
            // something in your column must be true
        }

        public String getName() {
            return this.name;
        }

        public boolean getState() {
            return this.state;
        }
    }
}
