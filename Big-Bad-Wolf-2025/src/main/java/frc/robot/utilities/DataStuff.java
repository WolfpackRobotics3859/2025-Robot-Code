package frc.robot.utilities;

import java.util.FormatterClosedException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants.LEVELS;

public class DataStuff extends SubsystemBase
{
    private static String[] columns  = new String[] {"LEFT OR RIGHT", "LEVEL", "FACE"};
    private static int currentColumn = 0;

    // LEFT | RIGHT
    private static String[] leftRight = new String[] {"LEFT", "RIGHT"};
    private static int currentSide = 0;

    // ONE | TWO | THREE | FOUR
    private static String[] level =  new String[] {"L1", "L2", "L3", "L4"};
    private static LEVELS[] levelValue = new LEVELS[] {LEVELS.ONE, LEVELS.TWO, LEVELS.THREE, LEVELS.FOUR};
    private static int currentLevel = 0;

    // F1 | F2 | F3 | F4 | F5 | F6
    private static String[] face = new String[] {"ONE", "TWO", "THREE", "FOUR", "FIVE", "SIX"};
    private static int currentFace = 0;

    public DataStuff()
    {
        UpdateEverything();
    }

    /**
     * This value is 0 indexed for now where level one is represented by integer 0.
     * @param level
     */
    public static void SetSelectedLevel(int level)
    {
        currentLevel = level;
        UpdateLevelSmartDashboard();
    }

    public static void SetSelectedFace(int face)
    {
        currentFace = face;
        UpdateFaceSmartdashboard();
    }

    public static void SetSelectedSide(int side)
    {
        currentSide = side;
        UpdateSideSmartDashboard();
    }

    public Command Down()
    {
        return this.runOnce(() -> IncrementSelection());
    }

    public Command Up()
    {
        return this.runOnce(() -> DecrementSelection());
    }

    public Command Right()
    {
        return this.runOnce(() -> IncrementColumn());
    }

    public Command Left()
    {
        return this.runOnce(() -> DecrementColumn());
    }

    /**
     * LEFT OR RIGHT
     * @return 0 for left, 1 for right
     */
    public static int GetCurrentSide()
    {
        return currentSide;
    }

    /**
     * FACES
     * @return 0 indexed
     */
    public static int GetCurrentFace()
    {
        return currentFace;
    }

    public static String GetSide()
    {
        return leftRight[currentSide];
    }

    public static LEVELS GetLevel()
    {
        return levelValue[currentLevel];
    }

    public static String GetFace()
    {
        return face[currentFace];
    }

    public static String GetCoralAlignmentPathName()
    {
        return GetFace() + "-" + GetSide() + "-ALIGN"; 
    }

    public static String GetCleanAlignmentPathName()
    {
        return GetFace() + "-CLEAN-ALIGN";
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
        if(currentSide == 1)
        {
            currentSide = 0;
        }
        else
        {
            currentSide += 1;
        }
        UpdateSideSmartDashboard();
    }

    private static void DecrementSide()
    {
        if(currentSide == 0)
        {
            currentSide = 1;
        }
        else
        {
            currentSide -= 1;
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
        SmartDashboard.putNumber("SELECTED SIDE: ", currentSide);
    }

    private static void IncrementLevel()
    {
        if(currentLevel == 3)
        {
            currentLevel = 0;
        }
        else
        {
            currentLevel += 1;
        }
        UpdateLevelSmartDashboard();
    }

    private static void DecrementLevel()
    {
        if(currentLevel == 0)
        {
            currentLevel = 3;
        }
        else
        {
            currentLevel -= 1;
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
        SmartDashboard.putNumber("SELECTED LEVEL: ", currentLevel);
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
        SmartDashboard.putNumber("CURRENT FACE: ", currentFace);
    }


    private void UpdateEverything()
    {
        UpdateColumnSmartDashboard();
        UpdateFaceSmartdashboard();
        UpdateLevelSmartDashboard();
        UpdateSideSmartDashboard();
    }

}
