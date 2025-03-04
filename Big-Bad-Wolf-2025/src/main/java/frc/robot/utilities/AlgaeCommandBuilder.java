package frc.robot.utilities;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAlgae;

public class AlgaeCommandBuilder
{
   private Shooter m_Shooter;
   private Elevator m_Elevator;
   private ShooterAlgae m_ShooterAlgae;

   private HashMap<String, Pair<PathPlannerPath, PathPlannerPath>> m_Paths;

   public AlgaeCommandBuilder(Shooter shooter, ShooterAlgae shooterAlgae, Elevator elevator)
   {
    this.m_Shooter = shooter;
    this.m_Elevator = elevator;
    this.m_ShooterAlgae = shooterAlgae;
    m_Paths  = new HashMap<String, Pair<PathPlannerPath, PathPlannerPath>>();
   }

   public void LoadAlignmentPaths(String[] alignmentPaths, String[] departurePaths)
   {
    try
    {
        int i = 0;
        for (String path : alignmentPaths)
        {
            PathPlannerPath loadedAlignmentPath = PathPlannerPath.fromPathFile(path);
            PathPlannerPath loadedDeparturePath = PathPlannerPath.fromPathFile(departurePaths[i]);
            m_Paths.put(path, new Pair<PathPlannerPath, PathPlannerPath>(loadedAlignmentPath, loadedDeparturePath));
            DataLogManager.log("Loaded Clean Path: " + path);
            DataLogManager.log("Loaded Departure Path: " + departurePaths[i]);
        }
        DataLogManager.log("Algae Command Builder: Alignment and departure paths successfully loaded.");
    }
    catch(Exception e)
    {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        DataLogManager.log("AlgaeCommandBuilder failed to load alignment paths.");
    }
   }

   public void BuildAllCleaningCommandsAndPushToSmartDashboard()
   {
    for (String key : this.m_Paths.keySet()) 
    {
        String[] splitKey = key.split("-");
        SmartDashboard.putData(splitKey[0] + "-" + "CLEAN", this.BuildAlgaeRetrievalCommand(key));
    }
   }

   public Command BuildAlgaeRetrievalCommand(String alignmentPath)
   {
    Command preparationCommand;
    String[] splitKey = alignmentPath.split("-");
    if((splitKey[0] == "ONE") || (splitKey[0] == "THREE") || (splitKey[0] == "FOUR"))
    {
        preparationCommand = new ParallelCommandGroup(this.m_Elevator.MoveToLevel(LEVELS.LOW_ALGAE), this.m_Shooter.MoveToAlgaeSweep(), this.m_ShooterAlgae.BeginSweepAlgae());
    }
    else
    {
        preparationCommand = new ParallelCommandGroup(this.m_Elevator.MoveToLevel(LEVELS.HIGH_ALGAE), this.m_Shooter.MoveToAlgaeSweep(), this.m_ShooterAlgae.BeginSweepAlgae());
    }

    DataLogManager.log("AlgaeCommandBuilder created clean for face " + splitKey[0]);
    
    return new SequentialCommandGroup(preparationCommand, 
                                      AutoBuilder.followPath(this.m_Paths.get(alignmentPath).getFirst()),
                                      this.m_ShooterAlgae.HoldAlgae(),
                                      AutoBuilder.followPath(this.m_Paths.get(alignmentPath).getSecond()),
                                      new ParallelCommandGroup(this.m_Shooter.StowShooter(), this.m_Elevator.MoveToLevel(LEVELS.CORAL_INTAKE))
                                     );
   }
}
