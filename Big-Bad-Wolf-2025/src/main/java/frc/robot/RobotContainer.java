// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralPlacer;
//import frc.robot.commands.MoveToScoringPosition;
import frc.robot.commands.autos.CoralAlgaeTopStart;
import frc.robot.commands.autos.DriveForward;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.constants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  // Store subsystems in a public manager so other objects can easily cache them.
  public static final SubsystemManager m_Manager = new SubsystemManager();
  
  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.MaxSpeed * 0.1).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static PathPlannerPath DriveForward;
  public static PathPlannerPath ReefToAlgae;
  public static PathPlannerPath TopAlgaeToProc;
  public static PathPlannerPath TopStartToReef;

  public PathPlannerAuto CoralAlgaeTopStart;
  
    public RobotContainer() throws FileVersionException, IOException, ParseException
  {
    this.configurationChooser(BUILD_TYPE.COMPETITION);
    this.configureAutoChooser();
 }

  public static SubsystemManager getSubsystemManager()
  {
    return m_Manager;
  }

  private void configurationChooser(BUILD_TYPE type)
  {
    switch(type)
    {
      case COMPETITION:
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Intake());
        m_Manager.addSubsystem(new Climb());
        m_Manager.addSubsystem(new Shooter());
        this.configureCompetitionBindings();
      break;
/*
      case DRIVETRAIN_DEBUG:
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        this.configureDrivetrainDebugBindings();
      break;

      case ELEVATOR_DEBUG:
        m_Manager.addSubsystem(new Elevator());
        this.configureElevatorDebugBindings();
      break;

      case SHOOTER_DEBUG:
        m_Manager.addSubsystem(new Shooter());
        this.configureShooterDebugBindings();
      break;

      case INTAKE_DEBUG:
        m_Manager.addSubsystem(new Intake());
        this.configureIntakeDebugBindings();
      break;
  */
      default:
        System.out.println("Did you mean to configure nothing? :( Sad Robot Face");
      break;
    }
      
    SmartDashboard.putString("Active Build", type.name());
    System.out.println("[Wolfpack] Current build is: " + type.name());
  }

  
  private void configureCompetitionBindings()
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();
    Climb climb =  m_Manager.getSubsystemOfType(Climb.class).get();
    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();

//testing controls ; ONE CONTROLLER
  //Main Driver Controller
    //drivetrain
    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    //elevator
    m_DriverController.povUp().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE));
    m_DriverController.povDown().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE));

    //climb
    m_DriverController.povLeft().whileTrue(new ClimbCommand(climb, 1));
    m_DriverController.povRight().whileTrue(new ClimbCommand(climb, -1));

    //algae intake
    m_DriverController.rightTrigger().whileTrue(intake.goToPositionThenRoll(IntakeConstants.INTAKE_WRIST_GROUND_POSITION, -3));

    //process algae
    //m_DriverController.a().whileTrue(intake.goToPositionThenRoll(6, 3));

    //shooter
    m_DriverController.leftBumper().whileTrue(shooter.applyWristVoltage(-1));//up
    m_DriverController.leftTrigger().whileTrue(shooter.applyWristVoltage(1));//down
    //m_DriverController.y().whileTrue(shooter.applyShooterVoltage(-2));//outake
    //m_DriverController.b().whileTrue(shooter.applyShooterVoltage(2));//intake

   
    //AUTOMATED SHOOTER NO ALIGNMENT
    m_DriverController.a().onTrue(new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_TWO, shooter));
    m_DriverController.b().onTrue(new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_THREE, shooter));
    m_DriverController.y().onTrue(new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_FOUR, shooter));    
    m_DriverController.x().onTrue(shooter.applyShooterVoltage(-2));


/* 
  //comp controls
  //Driver Controller
    //drivetrain
    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );
    //algae intake
    m_DriverController.rightTrigger().whileTrue(intake.goToPositionThenRoll(IntakeConstants.INTAKE_WRIST_GROUND_POSITION, -3));
    //process algae
    m_DriverController.a().whileTrue(intake.goToPositionThenRoll(6, 3));
    //algae cleaning
    m_DriverController.leftBumper().whileTrue(shooter.applyWristVoltage(-1, 0.015));//up
    m_DriverController.leftTrigger().whileTrue(shooter.applyWristVoltage(-1, 0.015));//down

  //CoDriver Controller
    //coral placer  
    //Move the robot to the left scoring position, then score coral L4
    m_CoDriverController.povLeft().and(m_CoDriverController.y()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_FOUR, shooter) // Score
        )
    );
    //Move the robot to the right scoring position, then score coral L4
    m_CoDriverController.povRight().and(m_CoDriverController.y()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_FOUR, shooter) // Score
        )
    );

    //Move the robot to the left scoring position, then score coral L3
    m_CoDriverController.povLeft().and(m_CoDriverController.b()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_THREE, shooter) // Score
        )
    );
    //Move the robot to the right scoring position, then score coral L3
    m_CoDriverController.povRight().and(m_CoDriverController.b()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_THREE, shooter) // Score
        )
    );

    //Move the robot to the left scoring position, then score coral L2
    m_CoDriverController.povLeft().and(m_CoDriverController.a()).onTrue(
      new SequentialCommandGroup(
          new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
          new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_TWO, shooter) // Score
      )
    );
    //Move the robot to the right scoring position, then score coral L2
    m_CoDriverController.povRight().and(m_CoDriverController.a()).onTrue(
      new SequentialCommandGroup(
          new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
          new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_TWO, shooter) // Score
      )
    );

    //trough placing
    m_CoDriverController.x().whileTrue(shooter.applyShooterVoltage(-2));

    //climb
    m_DriverController.leftBumper().whileTrue(new ClimbCommand(climb, 1));
    m_DriverController.rightBumper().whileTrue(new ClimbCommand(climb, -1));
*/


    /* USE THESE WHEN WE HAVE VISION AND TWO CONTROLLERS
  //CoDriver Controller
    //coral placer  
    //Move the robot to the left scoring position, then score coral L4
    m_CoDriverController.povLeft().and(m_CoDriverController.y()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_FOUR, shooter) // Score
        )
    );
    //Move the robot to the right scoring position, then score coral L4
    m_CoDriverController.povRight().and(m_CoDriverController.y()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_FOUR, shooter) // Score
        )
    );

    //Move the robot to the left scoring position, then score coral L3
    m_CoDriverController.povLeft().and(m_CoDriverController.b()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_THREE, shooter) // Score
        )
    );
    //Move the robot to the right scoring position, then score coral L3
    m_CoDriverController.povRight().and(m_CoDriverController.b()).onTrue(
        new SequentialCommandGroup(
            new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
            new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_THREE, shooter) // Score
        )
    );

    //Move the robot to the left scoring position, then score coral L2
    m_CoDriverController.povLeft().and(m_CoDriverController.a()).onTrue(
      new SequentialCommandGroup(
          new MoveToScoringPosition(drivetrain, vision, true), // Move to left position
          new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_TWO, shooter) // Score
      )
    );
    //Move the robot to the right scoring position, then score coral L2
    m_CoDriverController.povRight().and(m_CoDriverController.a()).onTrue(
      new SequentialCommandGroup(
          new MoveToScoringPosition(drivetrain, vision, false), // Move to right position
          new CoralPlacer(elevator, ElevatorConstants.ELEVATOR_LEVEL_TWO, shooter) // Score
      )
    );

    //trough placing
    m_CoDriverController.x().whileTrue(shooter.applyShooterVoltage(-2));
    */
  }
  

/* DEBUGS
  private void configureDrivetrainDebugBindings()
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

    m_DriverController.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_DriverController.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_DriverController.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_DriverController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    m_DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  private void configureElevatorDebugBindings()
  {
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    m_DriverController.rightBumper().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE));
    m_DriverController.leftBumper().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE));

    m_DriverController.povDown().onTrue(elevator.goToPosition(ElevatorConstants.ELEVATOR_ZERO_POSITION));
    m_DriverController.povRight().onTrue(elevator.goToPosition(ElevatorConstants.ELEVATOR_LEVEL_TWO));
    m_DriverController.povUp().onTrue(elevator.goToPosition(ElevatorConstants.ELEVATOR_LEVEL_THREE));
    m_DriverController.povLeft().onTrue(elevator.goToPosition(ElevatorConstants.ELEVATOR_LEVEL_FOUR));

    SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

    m_DriverController.a().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    m_DriverController.b().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    m_DriverController.y().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    m_DriverController.x().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));
    System.out.println("[Wolfpack] Elevator Debug bindings successfully configured.");
  }

  private void configureShooterDebugBindings()
  {
    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();

    m_DriverController.rightBumper().whileTrue(shooter.applyWristVoltage(1)); // FORWARD
    m_DriverController.leftBumper().whileTrue(shooter.applyWristVoltage(-1)); // REVERSE

    m_DriverController.a().whileTrue(shooter.applyShooterVoltage(3)); // SHOOT OUT
    m_DriverController.b().whileTrue(shooter.applyShooterVoltage(-3)); // INTAKE IN
    System.out.println("[Wolfpack] Shooter Debug bindings successfully configured.");
  }

  private void configureIntakeDebugBindings()
  {
    Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();

    m_DriverController.rightBumper().whileTrue(intake.applyWristVoltage(1)); // FORWARD
    m_DriverController.leftBumper().whileTrue(intake.applyWristVoltage(-1)); // REVERSE

    m_DriverController.a().whileTrue(intake.applyShooterVoltage(3)); // SHOOT OUT
    m_DriverController.b().whileTrue(intake.applyShooterVoltage(-3)); // INTAKE IN

    m_DriverController.y().onTrue(intake.goToIntakePosition());
    m_DriverController.x().onTrue(intake.goToStowPosition());

    m_DriverController.povUp().onTrue(intake.goToPositionThenRoll(7.43, 3));
  }
*/
  
  public void configureAutoChooser() 
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();
    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();

    //load in autos
    CoralAlgaeTopStart = new PathPlannerAuto("CoralAlgaeTopStart");
    //load in paths
        try {
          DriveForward = PathPlannerPath.fromPathFile("DriveForward");
          ReefToAlgae = PathPlannerPath.fromPathFile("ReefToAlgae");
          TopAlgaeToProc = PathPlannerPath.fromPathFile("TopAlgaeToProc");
          TopStartToReef = PathPlannerPath.fromPathFile("TopStartToReef");
      } catch (IOException | FileVersionException | ParseException e) {
          System.err.println("[ERROR] Failed to load PathPlanner paths: " + e.getMessage());
          e.printStackTrace();
      }
      
   //add auto options
    autoChooser.setDefaultOption("SIT_STAY", Commands.none()); //does nothing
   
        try {
          autoChooser.addOption("DRIVE_FORWARD", new DriveForward(drivetrain));
          autoChooser.addOption("CORAL_ALGAE_TOP_START", new CoralAlgaeTopStart(drivetrain, elevator, shooter, intake));
      } catch (IOException | FileVersionException | ParseException e) {
          System.err.println("[ERROR] Failed to load PathPlanner autos: " + e.getMessage());
          e.printStackTrace();
      }

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public Command getAutonomousCommand() 
  {
    Command selectedCommand = autoChooser.getSelected();
    System.out.println("Selected Auto: " + selectedCommand.getName());
    return selectedCommand;  
  }
}
