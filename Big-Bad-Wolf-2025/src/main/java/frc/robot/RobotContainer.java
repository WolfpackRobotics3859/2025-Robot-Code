// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeIntake.AlgaeIntakeGroundCommand;
import frc.robot.commands.AlgaeIntake.AlgaeIntakeProcessingCommand;
import frc.robot.commands.AlgaeIntake.AlgaeIntakeStowCommand;
import frc.robot.commands.CommandGroups.AlgaeCleanerBottomLevel;
import frc.robot.commands.CommandGroups.AlgaeCleanerIntakeToStow;
import frc.robot.commands.CommandGroups.AlgaeCleanerTopLevel;
import frc.robot.commands.groups.PlaceCoralOnReef;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorGoHome;
import frc.robot.commands.Elevator.ElevatorLevelTwo;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.commands.Elevator.ElevatorLevelThree;
import frc.robot.commands.Elevator.ElevatorLevelFour;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AlgaeCleaner;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CoralPlacer;

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

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public Command forwardDynamicCommand;
  public Command reverseDynamicCommand; 
  public Command forwardQuasistaticCommand;
  public Command reverseQuasistaticCommand;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    this.initializeSubsystems();
    this.configureBindings();
    this.configureCharacterizationBindings(); // REMOVE FROM COMPETITION BUILD (REMOVE_BEFORE_COMP)
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
    m_Manager.addSubsystem(new Climb());
  }

  private void configureBindings() 
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    AlgaeIntake algaeIntake = m_Manager.getSubsystemOfType(AlgaeIntake.class).get();
    AlgaeCleaner algaeCleaner = m_Manager.getSubsystemOfType(AlgaeCleaner.class).get();
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    CoralPlacer coralPlacer = m_Manager.getSubsystemOfType(CoralPlacer.class).get();
    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();


    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

      // Main driver controller button bindings
      m_DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
      m_DriverController.rightTrigger().whileTrue(new AlgaeIntakeGroundCommand(algaeIntake)).onChange(new AlgaeIntakeStowCommand(algaeIntake));
      m_DriverController.a().whileTrue(new AlgaeIntakeProcessingCommand(algaeIntake)).onChange(new AlgaeIntakeStowCommand(algaeIntake));
      // m_DriverController.leftTrigger().whileTrue(new ClimbCommandGroup(climb));

      m_DriverController.povUp().whileTrue(new AlgaeCleanerTopLevel(elevator, algaeCleaner)).onChange(new AlgaeCleanerIntakeToStow(elevator, algaeCleaner));
      m_DriverController.povDown().whileTrue(new AlgaeCleanerBottomLevel(elevator, algaeCleaner)).onChange(new AlgaeCleanerIntakeToStow(elevator, algaeCleaner));

      // Secondary driver controller button bindings
      m_CoDriverController.a().whileTrue(new PlaceCoralOnReef(coralPlacer, new ElevatorLevelTwo(elevator))).onChange(new ElevatorGoHome(elevator));
      m_CoDriverController.x().whileTrue(new PlaceCoralOnReef(coralPlacer, new ElevatorLevelThree(elevator))).onChange(new ElevatorGoHome(elevator));
      m_CoDriverController.y().whileTrue(new PlaceCoralOnReef(coralPlacer, new ElevatorLevelFour(elevator))).onChange(new ElevatorGoHome(elevator));

      m_CoDriverController.povUp().whileTrue(new ElevatorUp(elevator));
      m_CoDriverController.povDown().whileTrue(new ElevatorDown(elevator));
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

  public Command getAutonomousCommand() 
  {
      return Commands.print("No autonomous command configured");
  }
}