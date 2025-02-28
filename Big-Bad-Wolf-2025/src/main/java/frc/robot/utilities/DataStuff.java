package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataStuff extends SubsystemBase
{
    private static String[] columns  = new String[] {"LEFT OR RIGHT", "LEVEL", "FACE"};
    private static int currentColumn = 0;

    // LEFT | RIGHT
    private static String[] leftRight = new String[] {"LEFT", "RIGHT"};
    private static int currentSide = 0;

    // ONE | TWO | THREE | FOUR
    private static String[] level =  new String[] {"L1", "L2", "L3", "L4"};
    private static int currentLevel = 0;

    // F1 | F2 | F3 | F4 | F5 | F6
    private static String[] face = new String[] {"F1", "F2", "F3", "F4", "F5", "F6"};
    private static int currentFace = 0;

    public DataStuff()
    {
        // Intentionally Empty
    }

    public Command Up()
    {
        return this.runOnce(() -> IncrementSelection());
    }

    public Command Down()
    {
        return this.runOnce(() -> DecrementSelection());
    }

    public Command Left()
    {
        return this.runOnce(() -> IncrementColumn());
    }

    public Command Right()
    {
        return this.runOnce(() -> DecrementColumn());
    }

    public static int GetSide()
    {
        return currentSide;
    }

    public static int GetLevel()
    {
        return currentLevel;
    }

    public static int GetFace()
    {
        return currentFace;
    }

    public static void IncrementSelection()
    {
        if(currentColumn == 2)
        {
            currentColumn = 0;
        }
        else
        {
            currentColumn += 1;
        }
        UpdateColumnSmartDashboard();
    }

    public static void DecrementSelection()
    {
        if(currentColumn == 0)
        {
            currentColumn = 2;
        }
        else
        {
            currentColumn -= 1;
        }
        UpdateColumnSmartDashboard();
    }

    public static void IncrementColumn()
    {
        if(currentColumn == 0)
        {
            IncrementSide();
        }
        else if(currentColumn == 1)
        {
            IncrementLevel();
        }
        else
        {
            IncrementFace();
        }
    }

    public static void DecrementColumn()
    {
        if(currentColumn == 0)
        {
            DecrementSide();
        }
        else if(currentColumn == 1)
        {
            DecrementLevel();
        }
        else
        {
            DecrementFace();
        }
    }

    private static void UpdateColumnSmartDashboard()
    {
        if(currentColumn == 0)
        {
            SmartDashboard.putBoolean(columns[0], true);
            SmartDashboard.putBoolean(columns[1], false);
            SmartDashboard.putBoolean(columns[2], false);
        }
        else if(currentColumn == 1)
        {
            SmartDashboard.putBoolean(columns[0], false);
            SmartDashboard.putBoolean(columns[1], true);
            SmartDashboard.putBoolean(columns[2], false);
        }
        else if(currentColumn == 2)
        {
            SmartDashboard.putBoolean(columns[0], false);
            SmartDashboard.putBoolean(columns[1], false);
            SmartDashboard.putBoolean(columns[2], true);
        }
    }

    private static void IncrementSide()
    {
        if(currentFace == 1)
        {
            currentFace = 0;
        }
        else
        {
            currentFace += 1;
        }
        UpdateSideSmartDashboard();
    }

    private static void DecrementSide()
    {
        if(currentFace == 0)
        {
            currentFace = 1;
        }
        else
        {
            currentFace -= 1;
        }
        UpdateSideSmartDashboard();
    }

    private static void UpdateSideSmartDashboard()
    {
        if(currentSide == 0)
        {
            SmartDashboard.putBoolean(leftRight[0], true);
            SmartDashboard.putBoolean(leftRight[1], false);
        }
        else if(currentSide == 1)
        {
            SmartDashboard.putBoolean(leftRight[0], false);
            SmartDashboard.putBoolean(leftRight[1], true);
        }
    }

    private static void IncrementLevel()
    {
        if(currentFace == 3)
        {
            currentFace = 0;
        }
        else
        {
            currentFace += 1;
        }
        UpdateLevelSmartDashboard();
    }

    private static void DecrementLevel()
    {
        if(currentFace == 0)
        {
            currentFace = 3;
        }
        else
        {
            currentFace -= 1;
        }
        UpdateLevelSmartDashboard();
    }

    private static void UpdateLevelSmartDashboard()
    {
        if(currentLevel == 0)
        {
            SmartDashboard.putBoolean(level[0], true);
            SmartDashboard.putBoolean(level[1], false);
            SmartDashboard.putBoolean(level[2], false);
            SmartDashboard.putBoolean(level[3], false);
        }
        else if(currentLevel == 1)
        {
            SmartDashboard.putBoolean(level[0], false);
            SmartDashboard.putBoolean(level[1], true);
            SmartDashboard.putBoolean(level[2], false);
            SmartDashboard.putBoolean(level[3], false);
        }
        else if(currentLevel == 2)
        {
            SmartDashboard.putBoolean(level[0], false);
            SmartDashboard.putBoolean(level[1], false);
            SmartDashboard.putBoolean(level[2], true);
            SmartDashboard.putBoolean(level[3], false);
        }
        else if(currentLevel == 3)
        {
            SmartDashboard.putBoolean(level[0], false);
            SmartDashboard.putBoolean(level[1], false);
            SmartDashboard.putBoolean(level[2], false);
            SmartDashboard.putBoolean(level[3], true);
        }
    }

    private static void IncrementFace()
    {
        if(currentFace == 5)
        {
            currentFace = 0;
        }
        else
        {
            currentFace += 1;
        }
        UpdateFaceSmartdashboard();
    }

    private static void DecrementFace()
    {
        if(currentFace == 0)
        {
            currentFace = 5;
        }
        else
        {
            currentFace -= 1;
        }
        UpdateFaceSmartdashboard();
    }

    private static void UpdateFaceSmartdashboard()
    {
        if(currentFace == 0)
        {
            SmartDashboard.putBoolean(face[0], true);
            SmartDashboard.putBoolean(face[1], false);
            SmartDashboard.putBoolean(face[2], false);
            SmartDashboard.putBoolean(face[3], false);
            SmartDashboard.putBoolean(face[4], false);
            SmartDashboard.putBoolean(face[5], false);

        }
        else if(currentFace == 1)
        {
            SmartDashboard.putBoolean(face[0], false);
            SmartDashboard.putBoolean(face[1], true);
            SmartDashboard.putBoolean(face[2], false);
            SmartDashboard.putBoolean(face[3], false);
            SmartDashboard.putBoolean(face[4], false);
            SmartDashboard.putBoolean(face[5], false);
        }
        else if(currentFace == 2)
        {
            SmartDashboard.putBoolean(face[0], false);
            SmartDashboard.putBoolean(face[1], false);
            SmartDashboard.putBoolean(face[2], true);
            SmartDashboard.putBoolean(face[3], false);
            SmartDashboard.putBoolean(face[4], false);
            SmartDashboard.putBoolean(face[5], false);
        }
        else if(currentFace == 3)
        {
            SmartDashboard.putBoolean(face[0], false);
            SmartDashboard.putBoolean(face[1], false);
            SmartDashboard.putBoolean(face[2], false);
            SmartDashboard.putBoolean(face[3], true);
            SmartDashboard.putBoolean(face[4], false);
            SmartDashboard.putBoolean(face[5], false);
        }
        else if(currentFace == 4)
        {
            SmartDashboard.putBoolean(face[0], false);
            SmartDashboard.putBoolean(face[1], false);
            SmartDashboard.putBoolean(face[2], false);
            SmartDashboard.putBoolean(face[3], false);
            SmartDashboard.putBoolean(face[4], true);
            SmartDashboard.putBoolean(face[5], false);
        }
        else if(currentFace == 5)
        {
            SmartDashboard.putBoolean(face[0], false);
            SmartDashboard.putBoolean(face[1], false);
            SmartDashboard.putBoolean(face[2], false);
            SmartDashboard.putBoolean(face[3], false);
            SmartDashboard.putBoolean(face[4], false);
            SmartDashboard.putBoolean(face[5], true);
        }
    }




}
