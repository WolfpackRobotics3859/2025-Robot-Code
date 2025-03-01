package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class CoralCommandBuilder
{
   private Shooter m_Shooter;
   private Elevator m_Elevator;

   private HashMap<String, PathPlannerPath> m_AlignmentPaths;

  // private DataStuff m_DataStuff;

   public CoralCommandBuilder(Shooter shooter, Elevator elevator)
   {
    this.m_Shooter = shooter;
    this.m_Elevator = elevator;
    m_AlignmentPaths  = new HashMap<String, PathPlannerPath>();
   }

   public void LoadAlignmentPaths(String[] pathNames)
   {
    try
    {
        for (String path : pathNames)
        {
            PathPlannerPath loadedPath = PathPlannerPath.fromPathFile(path);
            m_AlignmentPaths.put(path, loadedPath);
            DataLogManager.log("Loaded Alignment Path: " + path);
        }
        DataLogManager.log("Coral Command Builder: Alignment paths successfully loaded.");
    }
    catch(Exception e)
    {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        DataLogManager.log("CoralCommandBuilder failed to load alignment paths.");
    }
   }

   public void BuildAllL2CommandsAndDeployToSmartdashboard()
   {
    for (String key : this.m_AlignmentPaths.keySet()) 
    {
        String[] splitKey = key.split("-");
        SmartDashboard.putData(splitKey[0] + "-" + splitKey[1] + "-2", this.BuildCoralDeploymentCommand(key, LEVELS.TWO));
    }
   }

   public void BuildAllL3CommandsAndDeployToSmartdashboard()
   {
    for (String key : this.m_AlignmentPaths.keySet()) 
    {
        String[] splitKey = key.split("-");
        SmartDashboard.putData(splitKey[0] + "-" + splitKey[1] + "-3", this.BuildCoralDeploymentCommand(key, LEVELS.THREE));
    }
   }

   public void BuildAllL4CommandsAndDeployToSmartdashboard()
   {
    for (String key : this.m_AlignmentPaths.keySet()) 
    {
        String[] splitKey = key.split("-");
        SmartDashboard.putData(splitKey[0] + "-" + splitKey[1] + "-4", this.BuildCoralDeploymentCommand(key, LEVELS.FOUR));
    }
   }

   private Command BuildCoralDeploymentCommand(String alignmentPath, LEVELS level)
   {
    Command preparationCommand;
    if(level == LEVELS.FOUR)
    {
        preparationCommand = AutoBuilder.followPath(m_AlignmentPaths.get(alignmentPath))
                                .alongWith(this.m_Elevator.MoveToLevel(level))
                                .alongWith(this.m_Shooter.PrepareToDeployCoralHigh());
        return preparationCommand.andThen(this.m_Shooter.PrepareToDeployCoralHigh());
    }
    
    preparationCommand = AutoBuilder.followPath(m_AlignmentPaths.get(alignmentPath))
                                .alongWith(this.m_Elevator.MoveToLevel(level))
                                .alongWith(this.m_Shooter.PrepareToDeployCoralLow());

    return preparationCommand.andThen(this.m_Shooter.DeployCoralLowSmiley());
   }

   public Command BuildCoralStandingDeployment(LEVELS level)
   {
    Command preparationCommand;
    if(level == LEVELS.FOUR)
    {
        preparationCommand = this.m_Elevator.MoveToLevel(level)
                                        .alongWith(this.m_Shooter.PrepareToDeployCoralHigh());
        return preparationCommand.andThen(this.m_Shooter.DeployCoralHighSmiley());
    }
    
    preparationCommand = this.m_Elevator.MoveToLevel(level)
                                        .alongWith(this.m_Shooter.PrepareToDeployCoralLow());    
    return preparationCommand.andThen(this.m_Shooter.DeployCoralLowSmiley());
   }
}
