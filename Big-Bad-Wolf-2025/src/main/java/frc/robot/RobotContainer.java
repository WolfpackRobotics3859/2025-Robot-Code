// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(TunerConstants.DRIVETRAIN_CONSTANTS, 250, TunerConstants.FRONT_LEFT,
                                                         TunerConstants.FRONT_RIGHT, TunerConstants.BACK_LEFT, TunerConstants.BACK_RIGHT);

  //contoller
  private final CommandXboxController controllerOne = new CommandXboxController(0); // primary controlelr
  private final Supplier<Double> m_PrimaryControllerLeftY = () -> -controllerOne.getLeftY() * m_Drivetrain.axisModifier;
  private final Supplier<Double> m_PrimaryControllerLeftX = () -> -controllerOne.getLeftX() * m_Drivetrain.axisModifier;
  private final Supplier<Double> m_PrimaryControllerRightX = () -> -controllerOne.getRightX();

  private SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. 
     * @throws ParseException 
     * @throws IOException 
     * @throws FileVersionException */
     
    public RobotContainer() throws FileVersionException, IOException, ParseException 
  {
    this.configureAutoChooser();
    this.configureBindings();
  }

  private void configureAutoChooser() throws FileVersionException, IOException, ParseException {

    autoChooser = AutoBuilder.buildAutoChooser();
        
    //if no paths are selected
    autoChooser.setDefaultOption(null, AutoBuilder.followPath(null));

    //paths
    PathPlannerPath algaetoProc = PathPlannerPath.fromPathFile("AlgaeToProc");
    PathPlannerPath blueReefToAlgae = PathPlannerPath.fromPathFile("BlueReefToAlgae");
    PathPlannerPath midBlueToReef = PathPlannerPath.fromPathFile("MidBlueToReef");
    PathPlannerPath redAlgaeToProc = PathPlannerPath.fromPathFile("RedAlgaeToProc");
    PathPlannerPath redReefToAlgae = PathPlannerPath.fromPathFile("RedReefToAlgae");
    PathPlannerPath redTopToReef = PathPlannerPath.fromPathFile("RedTopToReef");

    //autos
    List<PathPlannerPath> autoPathBlue = PathPlannerAuto.getPathGroupFromAutoFile("1Coral1AlgaeBlue");
    List<PathPlannerPath> autoPathRed = PathPlannerAuto.getPathGroupFromAutoFile("1Coral1AlgaeRed");


    //adding paths to autoChooser
    autoChooser.addOption("AlgaeToProc", AutoBuilder.followPath(algaetoProc));
    autoChooser.addOption("BlueReefToAlgae", AutoBuilder.followPath(blueReefToAlgae));
    autoChooser.addOption("MidBlueToReef", AutoBuilder.followPath(midBlueToReef));
    autoChooser.addOption("RedAlgaeToProc", AutoBuilder.followPath(redAlgaeToProc));
    autoChooser.addOption("RedReefToAlgae", AutoBuilder.followPath(redReefToAlgae));
    autoChooser.addOption("RedTopToReef", AutoBuilder.followPath(redTopToReef));

    //adding auto to autoChooser
    autoChooser.addOption("1Coral1AlgaeBlue", AutoBuilder.followPath((PathPlannerPath) autoPathBlue));
    autoChooser.addOption("1Coral1AlgaeRed", AutoBuilder.followPath((PathPlannerPath) autoPathRed));

    /** 
    Psuedocode for auto routines:

    Autonomous Routine 1: "Score and Move"
    Start at initial position
    Drive to scoring position using "MidBlueToReef" path
    Score Game Piece
    Drive to game piece using "BlueReefToAlgae" path
    Pick up game piece
    Drive to second scoring position using "AlgaeToProc" path
    Score Game Piece
    End

    it would look like:

    PathPlannerPath scorePath = PathPlannerPath.fromPathFile("MidBlueToReef");
    PathPlannerPath pickupPath = PathPlannerPath.fromPathFile("BlueReefToAlgae");
    PathPlannerPath secondScorePath = PathPlannerPath.fromPathFile("AlgaeToProc");

    Command autoRoutine = Commands.sequence(
    AutoBuilder.followPath(scorePath),
    new ScoreGamePieceCommand(),      //this will be created and actually be code that scores a game piece
    AutoBuilder.followPath(pickupPath),
    new PickupGamePieceCommand(),     //this will be created and acutally be code that picks up the game piece
    AutoBuilder.followPath(secondScorePath),
    new SecondScoreGamePieceCommand()   //this will be created and actually be code that scores a game piece
    );
*/ 

    SmartDashboard.putData("Auto Chooser", autoChooser);
}

  public void startTeleopCommands()
  {
    m_Drivetrain.getDefaultCommand().schedule();
  }

  private void configureBindings() 
  {
    m_Drivetrain.setDefaultCommand(new Drive(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_PrimaryControllerRightX));
  }
  
  
  public Command getAutonomousCommand() throws FileVersionException, IOException, ParseException 
  {
    return autoChooser.getSelected();
  }
}
