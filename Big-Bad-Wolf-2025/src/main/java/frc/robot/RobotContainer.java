// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;

import org.json.simple.parser.ParseException;


import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeIntakeGroundCommand;
import frc.robot.commands.AlgaeIntakeProcessingCommand;
import frc.robot.commands.ElevatorLevelTwo;
import frc.robot.commands.groups.PlaceCoralOnReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AlgaeCleaner;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CoralPlacer;
import frc.robot.commands.autos.MidBlueToReef;
import frc.robot.commands.autos.RedTopToReef;


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
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public Command forwardDynamicCommand;
  public Command reverseDynamicCommand; 
  public Command forwardQuasistaticCommand;
  public Command reverseQuasistaticCommand;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() throws FileVersionException, IOException, ParseException 
  {
    this.initializeSubsystems();
    this.configureBindings();
    this.configureCharacterizationBindings(); // REMOVE FROM COMPETITION BUILD (REMOVE_BEFORE_COMP)
    configureAutoCommands();
    configureAutoChooser();

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  public static SubsystemManager getSubsystemManager()
  {
    return m_Manager;
  }

  private void initializeSubsystems()
  {
    m_Manager.addSubsystem(TunerConstants.createDrivetrain());
    m_Manager.addSubsystem(new Elevator());
    m_Manager.addSubsystem(new AlgaeIntake());
    m_Manager.addSubsystem(new AlgaeCleaner());
    m_Manager.addSubsystem(new CoralPlacer());

    SmartDashboard.putData("Auto Selector", autoChooser);
    m_Manager.addSubsystem(new Climb());
  }

  private void configureBindings() 
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

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

  /*
   * Configures the bindings for characterization subroutines. Shouldn't be bound in a competition environment.
   */
  private void configureCharacterizationBindings()
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    forwardDynamicCommand = drivetrain.sysIdDynamic(Direction.kForward);
    reverseDynamicCommand = drivetrain.sysIdDynamic(Direction.kReverse);
    forwardQuasistaticCommand = drivetrain.sysIdQuasistatic(Direction.kForward);
    reverseQuasistaticCommand = drivetrain.sysIdQuasistatic(Direction.kReverse);

    SmartDashboard.putData("Forward Dynamic", this.forwardDynamicCommand);
    SmartDashboard.putData("Reverse Dynamic", this.reverseDynamicCommand);
    SmartDashboard.putData("Forward Quasistatic", this.forwardQuasistaticCommand);
    SmartDashboard.putData("Reverse Quasistatic", this.reverseQuasistaticCommand); 
  }

  private void configureAutoCommands() throws FileVersionException, IOException, ParseException
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();

    NamedCommands.registerCommand("MID_BLUE_TO_REEF", new MidBlueToReef(drivetrain, elevator));
    //NamedCommands.registerCommand("RED_TOP_TO_REEF", new RedTopToReef(m_Drivetrain, m_Elevator));
  }

  private void configureAutoChooser() throws FileVersionException, IOException, ParseException
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();


    //default doing nothing
    autoChooser.setDefaultOption("SIT_STAY", Commands.none());

    //autos
    autoChooser.addOption("MID_BLUE_TO_REEF", new MidBlueToReef(drivetrain, elevator));
    //autoChooser.addOption("RED_TOP_TO_REEF", new RedTopToReef(m_Drivetrain, m_Elevator));
    
  }

  public Command getAutonomousCommand() 
  {
    Command selectedCommand = autoChooser.getSelected();
    System.out.println("Selected Auto: " + selectedCommand.getName());
    return selectedCommand;
  }
}
