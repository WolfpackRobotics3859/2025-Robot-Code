// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class DriveForward extends SequentialCommandGroup 
{
  
  public DriveForward(CommandSwerveDrivetrain drivetrain) throws FileVersionException, IOException, ParseException 
  {
    setName("DRIVE_FORWARD");
    addRequirements(drivetrain);

    // Paths used
    PathPlannerPath driveForward = RobotContainer.DriveForward;
    //debug check for paths
    if (driveForward == null)
    {
      System.out.println("Error: DriveForward path is not initialized!");
      return;    
    }

    // Create command sequence
    addCommands(
        AutoBuilder.followPath(driveForward)
      );

    System.out.println("DriveForward Path Initialized");
    
  }    
}


