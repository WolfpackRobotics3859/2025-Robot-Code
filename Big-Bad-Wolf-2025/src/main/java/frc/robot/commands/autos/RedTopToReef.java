// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;


public class RedTopToReef extends SequentialCommandGroup 
{

  public RedTopToReef(CommandSwerveDrivetrain drivetrain, Elevator elevator) throws FileVersionException, IOException, ParseException 
  {
    setName("RED_TOP_TO_REEF");
    addRequirements(drivetrain, elevator);
    
    //paths used
    PathPlannerPath scorePath = PathPlannerPath.fromPathFile("redTopToReef");
    PathPlannerPath pickupPath = PathPlannerPath.fromPathFile("RedReefToAlgae");
    PathPlannerPath secondScorePath = PathPlannerPath.fromPathFile("RedAlgaeToProc");

    addCommands
    (
      AutoBuilder.followPath(scorePath),
      //new ScoreGamePieceCommand(),      //this will be created and actually be code that scores a game piece
      AutoBuilder.followPath(pickupPath),
      //new PickupGamePieceCommand(),     //this will be created and acutally be code that picks up the game piece
      AutoBuilder.followPath(secondScorePath),
      //new SecondScoreGamePieceCommand()   //this will be created and actually be code that scores a game piece
      AutoBuilder.followPath(null)  //delete once we have game piece commands
    );
  }
}
