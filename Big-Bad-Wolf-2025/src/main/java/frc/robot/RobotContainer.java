// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Global;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DataSelector;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.dataSelector.DataSelectorHelper;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Automation;
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
          .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    this.configurationChooser(Global.ACTIVE_BUILD);
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
        m_Manager.addSubsystem(new DataSelector(DataSelectorHelper.HEADER_COLUMN));
        m_Manager.addSubsystem(new Automation(m_Manager));
        this.configureCompetitionBindings();
      break;

      case DRIVETRAIN_DEBUG:
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        this.configureDrivetrainDebugBindings();
      break;

      case ELEVATOR_DEBUG:
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        this.configureElevatorDebugBindings();
      break;

      case SHOOTER_DEBUG:
        m_Manager.addSubsystem(new Shooter());
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        this.configureShooterDebugBindings();
      break;

      case INTAKE_DEBUG:
        m_Manager.addSubsystem(new Intake());
        this.configureIntakeDebugBindings();
      break;

      case CLIMB_DEBUG:
        m_Manager.addSubsystem(new Climb());
        this.configureClimbDebugBindings();
      break;

      case AUTOMATION_DEBUG:
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        m_Manager.addSubsystem(new Automation(m_Manager));
        this.configureAutomationDebugBindings();
      break;
      
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
    SmartDashboard.putData(elevator);

    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    SmartDashboard.putData(shooter);

    Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();
    SmartDashboard.putData(intake);

    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

    DataSelector dataSelector =m_Manager.getSubsystemOfType(DataSelector.class).get();
    dataSelector.addColumn(DataSelectorHelper.LEFT_RIGHT_CLEAN_COLUMN);
    dataSelector.addColumn(DataSelectorHelper.LEVELS_COLUMN);
    dataSelector.addColumn(DataSelectorHelper.REEF_FACE_SELECTION_COLUMN);
    SmartDashboard.putData(dataSelector);

    Automation automation = m_Manager.getSubsystemOfType(Automation.class).get();
    SmartDashboard.putData(automation);

    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed *0.8)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed * 0.8)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    m_DriverController.rightTrigger().onTrue(automation.CoralPlacementRoutine())
                                     .onFalse(automation.ResetTheStuffs());

    m_DriverController.leftTrigger().onTrue(shooter.IntakeCoral().alongWith(elevator.MoveToLevel(LEVELS.CORAL_INTAKE)))
                                    .onFalse(automation.ResetTheStuffs());

    m_DriverController.rightBumper().onTrue(elevator.MoveToLevel(LEVELS.ALGAE_PROCESS)
                                                    .andThen(shooter.SetWristPosition(ShooterConstants.WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION))
                                                    .andThen(shooter.ProcessAlgae()))
                                    .onFalse(shooter.StopAlgae()
                                                    .andThen(automation.ResetTheStuffs()));
          
    m_DriverController.leftBumper().onTrue(automation.CleanAlgae())
                                    .onFalse(automation.ResetTheStuffs());

    m_CoDriverController.leftBumper().onTrue(dataSelector.toggleUpColumn());
    m_CoDriverController.rightBumper().onTrue(dataSelector.toggleDownColumn());
    m_CoDriverController.povUp().onTrue(dataSelector.shiftColumnCategoryLeft());
    m_CoDriverController.povDown().onTrue(dataSelector.shiftColumnCategoryRight());

    m_CoDriverController.start().onTrue(automation.ToggleVision());
    m_CoDriverController.y().onTrue(elevator.ZeroElevator());
    m_CoDriverController.x().whileTrue(elevator.ApplyVoltage(0).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    m_CoDriverController.a().whileTrue(climb.setClimbVoltage(() -> -m_CoDriverController.getRawAxis(2)*10));
  }

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

    //m_DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  private void configureElevatorDebugBindings()
  {
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

    m_DriverController.start().onTrue(elevator.ZeroElevator());
    m_DriverController.povLeft().onTrue(elevator.MoveToLevel(LEVELS.HOME));
    m_DriverController.povUp().onTrue(elevator.ApplyVoltage(3)).onFalse(elevator.ApplyVoltage(0));
    m_DriverController.povDown().onTrue(elevator.ApplyVoltage(-0.8)).onFalse(elevator.ApplyVoltage(0));

    SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

    m_DriverController.a().onTrue(elevator.MoveToLevel(LEVELS.ONE));
    m_DriverController.b().onTrue(elevator.MoveToLevel(LEVELS.TWO));
    m_DriverController.y().onTrue(elevator.MoveToLevel(LEVELS.THREE));
    m_DriverController.x().onTrue(elevator.MoveToLevel(LEVELS.FOUR));

    // m_DriverController.a().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    // m_DriverController.b().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    // m_DriverController.y().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    // m_DriverController.x().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    System.out.println("[Wolfpack] Elevator Debug bindings successfully configured.");
  }

  private void configureShooterDebugBindings()
  {
    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    SmartDashboard.putData(shooter);

    // m_DriverController.a().onTrue(shooter.StowShooter());
    // m_DriverController.b().onTrue(shooter.IntakeAlgae()).onFalse(shooter.StowAndHoldAlgae());
    // m_DriverController.y().onTrue(shooter.PrepareToDeployCoralLow());
    // m_DriverController.x().onTrue(shooter.DeployCoral()).onFalse(shooter.StowShooter());

    // m_DriverController.povUp().onTrue(shooter.IntakeCoral()).onFalse(shooter.StowShooter());
    // m_DriverController.povDown().onTrue(shooter.ProcessAlgae()).onFalse(shooter.StowShooter());

    System.out.println("[Wolfpack] Shooter Debug bindings successfully configured.");
  }

  private void configureIntakeDebugBindings()
  {
    Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();
    SmartDashboard.putData(intake);


    m_DriverController.a().onTrue(intake.IntakeRoutine()).onFalse(intake.StowIntake());
  }

  private void configureClimbDebugBindings()
  {
    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

   // climb.setDefaultCommand(climb.setVoltage(() -> -m_DriverController.getRawAxis(5)*10));
  }

  private void configureAutomationDebugBindings()
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    
    m_DriverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d())));

    m_DriverController.a().toggleOnFalse(new PathPlannerAuto("New New Auto"));
    m_DriverController.b().toggleOnFalse(new PathPlannerAuto("SIX-LEFT-AUTO"));
    m_DriverController.y().toggleOnFalse(new PathPlannerAuto("SIX-RIGHT-AUTO"));
    
    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
