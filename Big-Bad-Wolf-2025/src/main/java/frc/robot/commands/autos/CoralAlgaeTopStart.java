// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import frc.robot.RobotContainer;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class CoralAlgaeTopStart extends SequentialCommandGroup 
{

  public CoralAlgaeTopStart(CommandSwerveDrivetrain drivetrain, Elevator elevator, Shooter shooter, Intake intake) throws FileVersionException, IOException, ParseException 
  {
    setName("CORAL_ALGAE_TOP_START");
    addRequirements(drivetrain, elevator, shooter, intake);

    //paths used
    PathPlannerPath scorePath = RobotContainer.TopStartToReef;
    PathPlannerPath pickupPath = RobotContainer.ReefToAlgae;
    PathPlannerPath secondScorePath = RobotContainer.TopAlgaeToProc;

    addCommands
    (
      AutoBuilder.followPath(scorePath),            //goes to reef
      elevator.goToPosition(ElevatorConstants.ELEVATOR_LEVEL_TWO),         //raises elevator
      shooter.applyWristVoltage(-3),        //wrist in shooting position
      new WaitCommand(1.5),
      shooter.applyShooterVoltage(ShooterConstants.SHOOTER_CORAL_DEPLOY_VOLTAGE),   //scores coral 
      new WaitCommand(0.5),                                                        
      //wait  can be removed when we have sensors that tell the shooter to stop
      shooter.applyWristVoltage(3),    //stow wrist
      new WaitCommand(1.5),
      elevator.goToPosition(ElevatorConstants.ELEVATOR_ZERO_POSITION),    //lowers elevator
      AutoBuilder.followPath(pickupPath),     //goes to algae
      intake.goToPositionThenRoll(IntakeConstants.INTAKE_WRIST_GROUND_POSITION, -3),    //intake algae  
      new WaitCommand(1),
      intake.goToPositionThenRoll(IntakeConstants.INTAKE_WRIST_STOW_POSITION, 0),   //stow algae
      AutoBuilder.followPath(secondScorePath),    //go to processor
      intake.goToPositionThenRoll(6, 3)     //processes algae
    );
  }
}
