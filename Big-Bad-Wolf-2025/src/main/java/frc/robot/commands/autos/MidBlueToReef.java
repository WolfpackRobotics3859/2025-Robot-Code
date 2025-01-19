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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class MidBlueToReef extends SequentialCommandGroup {

  public MidBlueToReef(CommandSwerveDrivetrain drivetrain, Elevator elevator  ) throws FileVersionException, IOException, ParseException 
  {
    //paths used
    PathPlannerPath reefPath = PathPlannerPath.fromPathFile("MidBlueToReef");

    System.out.println("Loaded path: " + reefPath);
    addCommands
    (
        AutoBuilder.followPath(reefPath)
    );
  }
}

