package frc.robot.utilities.dataSelector;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Column 
{    
    private String columnName;

    private int currentOptionIndex;
    private Option currentOption;

    List<Option> options;

    public Column(ColumnBuilder columnBuilder) {
        this.columnName = columnBuilder.columnName;
        this.options = columnBuilder.options;
        this.currentOption = options.get(0);
        this.currentOptionIndex = 0;
    }

    public int getCurrentOptionIndex() {
        return this.currentOptionIndex;
    }

    public String getName() {
        return this.columnName;
    }

    public Option getCurrentOption() {
        return this.currentOption;
    }

    public boolean incrementColumn() {
        if(currentOptionIndex + 1 < options.size())
        {
            currentOption.setState(false);

            currentOptionIndex ++;

            currentOption = options.get(currentOptionIndex);
            currentOption.setState(true);
            return true;
        }
        return false;
    }

    public boolean decrementColumn() 
    {
        if(currentOptionIndex > 0) 
        {
            currentOption.setState(false);

            currentOptionIndex --;

            currentOption = options.get(currentOptionIndex);
            currentOption.setState(true);
            return true;
        }
        return false;
    }

    public List<Option> getOptions() {
        return options;
    }

    public Column addOption(String name) {
        Option option;
        if(currentOption == null) {
            option = new Option(name, true);
            currentOptionIndex = 0;
            currentOption = option;
        } else {
            option = new Option(name);
        }
        options.add(option);
        return this; 
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

        public ColumnBuilder addFirstOption(String optionName, boolean isFirstOption) {
            options.add(new Option(optionName, isFirstOption));
            return this;
        }

        public ColumnBuilder addOption(String optionName) {
            options.add(new Option(optionName));
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

        public Option(String name, boolean firstValueOfList) {
            this.name = name;
            this.state = firstValueOfList;
            SmartDashboard.putBoolean(name, firstValueOfList);
        }

        public void setState(boolean state) {
            this.state = state;
            SmartDashboard.putBoolean(this.name, state);
        }

        public String getName() {
            return this.name;
        }

        public boolean getState() {
            return this.state;
        }
    }
}
