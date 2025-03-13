// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Global;
import frc.robot.constants.PathConstants;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAlgae;
import frc.robot.subsystems.ShooterCoral;
import frc.robot.utilities.AlgaeCommandBuilder;
import frc.robot.utilities.CoralCommandBuilder;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.PackLog;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Climb;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  private static final SubsystemManager m_Manager = new SubsystemManager();
  
  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
          .withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private CoralCommandBuilder commandBuilder;
  private AlgaeCommandBuilder algaeCommandBuilder;

  private final PackLog m_PackLog;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    this.m_PackLog = new PackLog("RobotContainer");
    this.configurationChooser(Global.ACTIVE_BUILD);
  }

  public void InitializeDefaultCommands()
  {
    // Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    // SmartDashboard.putData(elevator);

    // Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    // SmartDashboard.putData(shooter);

    // elevator.MoveToLevel(LEVELS.HOME).schedule();
    // shooter.StowShooter().schedule();
    this.m_PackLog.Log("Default commands scheduled.");
  }

  private void configurationChooser(BUILD_TYPE type)
  {
    this.m_PackLog.Log("Beginning configuration.");
    switch(type)
    {
      case COMPETITION:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        m_Manager.addSubsystem(new ShooterCoral());
        m_Manager.addSubsystem(new ShooterAlgae());
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Climb());
        m_Manager.addSubsystem(new Shooter());
        m_Manager.addSubsystem(new DataStuff());
        this.configureCompetitionBindingsV2();
      break;

      case COMPETITION_NO_VISION:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        m_Manager.addSubsystem(new ShooterCoral());
        m_Manager.addSubsystem(new ShooterAlgae());
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Climb());
        m_Manager.addSubsystem(new Shooter());
        m_Manager.addSubsystem(new DataStuff());
        this.configureCompetitionNoVisionBindings();
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

      case CLIMB_DEBUG:
        m_Manager.addSubsystem(new Climb());
        this.configureClimbDebugBindings();
      break;

      case ELEVATOR_SHOOTER_DEBUG:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Shooter());
        m_Manager.addSubsystem(new ShooterCoral());
        this.configureElevatorShooterDebugBindings();
      break;
      
      default:
        System.out.println("Did you mean to configure nothing? :( Sad Robot Face");
      break;
    }

    this.m_PackLog.Log("Configuration Complete for ACTIVE BUILD: " + type.name());
    SmartDashboard.putString("Active Build", type.name());
  }

  private SendableChooser<Command> m_Chooser;
  private void configureCompetitionBindingsV2()
  {
    Drivetrain drivetrain = m_Manager.getSubsystemOfType(Drivetrain.class).get();

    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    SmartDashboard.putData(shooter);

    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

    DataStuff dataStuff = m_Manager.getSubsystemOfType(DataStuff.class).get();
    SmartDashboard.putData(dataStuff);

    ShooterCoral shooterCoral = m_Manager.getSubsystemOfType(ShooterCoral.class).get();
    SmartDashboard.putData(shooterCoral);

    ShooterAlgae shooterAlgae = m_Manager.getSubsystemOfType(ShooterAlgae.class).get();
    SmartDashboard.putData(shooterAlgae);

    drivetrain.setDefaultCommand
    (
        drivetrain.DefaultDrive(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX(), () -> m_DriverController.getRightX())
    );

    shooterAlgae.setDefaultCommand(shooterAlgae.HoldAlgae());

    m_DriverController.rightTrigger().onTrue(new ParallelCommandGroup(drivetrain.AlignCoralNoEnd(), shooter.MoveToSelectedShot(), elevator.MoveToSelectorLevel()))
                                     .onFalse(shooter.StowShooter().andThen(elevator.MoveToLevel(LEVELS.HOME)));

    m_DriverController.leftTrigger().whileTrue(new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(),
                                                                         elevator.MoveToLevel(LEVELS.CORAL_INTAKE),
                                                                         shooter.MoveToIntake()))
                                    .onFalse(shooter.StowShooter().andThen(elevator.MoveToLevel(LEVELS.HOME)));

    m_DriverController.a().onTrue(new WaitCommand(0.1).andThen(shooterCoral.DeployCoralRoutine()))
                          .onFalse(shooterCoral.StopCoral());

    m_CoDriverController.leftBumper().onTrue(dataStuff.Left().ignoringDisable(true));
    m_CoDriverController.rightBumper().onTrue(dataStuff.Right().ignoringDisable(true));
    m_CoDriverController.povUp().onTrue(dataStuff.Up().ignoringDisable(true));
    m_CoDriverController.povDown().onTrue(dataStuff.Down().ignoringDisable(true));

    //climb wrist
    //m_CoDriverController.x().whileTrue(climb.setClimbVoltage(ClimbConstants.CLIMB_WRIST_VOLTAGE)).onFalse(climb.setClimbVoltage(0));
    //climb wheels
    //m_CoDriverController.b().whileTrue(climb.setRollerVoltage(ClimbConstants.CLIMB_ROLLER_VOLTAGE)).onFalse(climb.setRollerVoltage(0));
    //m_CoDriverController.b().whileTrue(climb.setLatchVoltage(-2)).onFalse(climb.setLatchVoltage(0));

    NamedCommands.registerCommand("ElevatorHome", elevator.MoveToLevel(LEVELS.HOME));
    NamedCommands.registerCommand("CoralTwoDeploy", new ParallelCommandGroup(elevator.MoveToLevel(LEVELS.TWO), shooter.MoveToDeployLow()));
    NamedCommands.registerCommand("CoralThreeDeploy", new ParallelCommandGroup(elevator.MoveToLevel(LEVELS.THREE), shooter.MoveToDeployLow()));
    NamedCommands.registerCommand("CoralFourDeploy", new ParallelCommandGroup(elevator.MoveToLevel(LEVELS.FOUR), shooter.MoveToDeployHigh()));
    NamedCommands.registerCommand("DeployCoral", shooterCoral.DeployCoralRoutine());

    NamedCommands.registerCommand("Align4L", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(0, 4)));
    NamedCommands.registerCommand("Align4R", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(1, 4)));
    NamedCommands.registerCommand("Align5L", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(0, 5)));
    NamedCommands.registerCommand("Align5R", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(1, 5)));
    NamedCommands.registerCommand("Align6L", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(0, 6)));
    NamedCommands.registerCommand("Align6R", new ParallelDeadlineGroup(new WaitCommand(0.75), drivetrain.AlignToFace(1, 6)));

    NamedCommands.registerCommand("ShooterLowPosition", new ParallelCommandGroup(shooter.MoveToDeployLow(), elevator.MoveToLevel(LEVELS.ZERO)));

    NamedCommands.registerCommand("StartIntake", new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(), elevator.MoveToLevel(LEVELS.CORAL_INTAKE), shooter.MoveToIntake()));
    
    try 
    {
      m_Chooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(m_Chooser);
     } 
     catch (Exception e) 
     {
       DriverStation.reportError("Failed to load autonomous chooser.", e.getStackTrace());
       e.printStackTrace();
     }
  }


  private void configureCompetitionBindings()
  {
    Drivetrain drivetrain = m_Manager.getSubsystemOfType(Drivetrain.class).get();

    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    SmartDashboard.putData(shooter);

    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

    DataStuff dataStuff = m_Manager.getSubsystemOfType(DataStuff.class).get();
    SmartDashboard.putData(dataStuff);

    ShooterCoral shooterCoral = m_Manager.getSubsystemOfType(ShooterCoral.class).get();
    SmartDashboard.putData(shooterCoral);

    ShooterAlgae shooterAlgae = m_Manager.getSubsystemOfType(ShooterAlgae.class).get();
    SmartDashboard.putData(shooterAlgae);

    commandBuilder = new CoralCommandBuilder(shooter, shooterCoral, elevator);
    commandBuilder.LoadAlignmentPaths(PathConstants.ALIGNMENT_PATHS);

    algaeCommandBuilder = new AlgaeCommandBuilder(shooter, shooterAlgae, elevator);
    algaeCommandBuilder.LoadAlignmentPaths(PathConstants.CLEAN_PATHS, PathConstants.CLEAN_DEPARTURE_PATHS);

    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed *0.8)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed * 0.8)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    m_DriverController.rightTrigger().onTrue(new InstantCommand(() -> commandBuilder.BuildCoralDeploymentCommand(DataStuff.GetCoralAlignmentPathName(), DataStuff.GetLevel()).schedule()))
                                     .onFalse(new ParallelCommandGroup(shooterCoral.StopCoral(), elevator.MoveToLevel(LEVELS.HOME), shooter.StowShooter()));

    m_DriverController.leftTrigger().whileTrue(new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(),
                                                                         elevator.MoveToLevel(LEVELS.CORAL_INTAKE),
                                                                         shooter.MoveToIntake()));

    m_DriverController.leftBumper().onTrue(new InstantCommand(() -> algaeCommandBuilder.BuildAlgaeRetrievalCommand(DataStuff.GetCleanAlignmentPathName()).schedule()))
                                   .onFalse(new ParallelCommandGroup(shooterAlgae.HoldAlgae(), elevator.MoveToLevel(LEVELS.HOME), shooter.StowShooter()));

    m_DriverController.rightBumper().whileTrue(new ParallelCommandGroup(elevator.MoveToLevel(LEVELS.ALGAE_PROCESS), shooter.MoveToAlgaeSweep())
                                                  .andThen(shooterAlgae.DeployAlgae())
                                             )
                                             .onFalse(new ParallelCommandGroup(shooter.StowShooter(), elevator.MoveToLevel(LEVELS.CORAL_INTAKE)));

    m_CoDriverController.leftBumper().onTrue(dataStuff.Left().ignoringDisable(true));
    m_CoDriverController.rightBumper().onTrue(dataStuff.Right().ignoringDisable(true));
    m_CoDriverController.povUp().onTrue(dataStuff.Up().ignoringDisable(true));
    m_CoDriverController.povDown().onTrue(dataStuff.Down().ignoringDisable(true));

    m_CoDriverController.a().onTrue(shooterAlgae.DeployAlgae())
                            .onFalse(shooterAlgae.StopAlgae());

    m_CoDriverController.y().onTrue(elevator.ZeroElevator());


    //climb wrist
    m_CoDriverController.x().whileTrue(climb.setClimbVoltage(ClimbConstants.CLIMB_WRIST_VOLTAGE)).onFalse(climb.setClimbVoltage(0));
    //climb wheels
    m_CoDriverController.b().whileTrue(climb.setRollerVoltage(ClimbConstants.CLIMB_ROLLER_VOLTAGE)).onFalse(climb.setRollerVoltage(0));
    //m_CoDriverController.b().whileTrue(climb.setLatchVoltage(-2)).onFalse(climb.setLatchVoltage(0));


    NamedCommands.registerCommand("CoralTwoDeploy", new ParallelCommandGroup(elevator.MoveToLevel(LEVELS.TWO), shooter.MoveToDeployLow()));
    NamedCommands.registerCommand("StartIntake", new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(), elevator.MoveToLevel(LEVELS.CORAL_INTAKE), shooter.MoveToIntake()));
    NamedCommands.registerCommand("DeployCoral", shooterCoral.DeployCoralRoutine());

    NamedCommands.registerCommand("LevelThreeDeploy", commandBuilder.BuildCoralStandingDeployment(LEVELS.THREE));

    SmartDashboard.putData("Level One", commandBuilder.BuildCoralStandingDeployment(LEVELS.ONE));
    
    SmartDashboard.putData("Level Two", commandBuilder.BuildCoralStandingDeployment(LEVELS.TWO));
    
    SmartDashboard.putData("Level Three", commandBuilder.BuildCoralStandingDeployment(LEVELS.THREE));
    
    try 
    {
      m_Chooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(m_Chooser);
     } 
     catch (Exception e) 
     {
       DriverStation.reportError("Failed to load autonomous chooser.", e.getStackTrace());
       e.printStackTrace();
     }
  }

  private void configureCompetitionNoVisionBindings()
  {
    // Empty for now
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

    SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

    m_DriverController.a().onTrue(elevator.MoveToLevel(LEVELS.ONE));
    m_DriverController.b().onTrue(elevator.MoveToLevel(LEVELS.TWO));
    m_DriverController.y().onTrue(elevator.MoveToLevel(LEVELS.THREE));
    m_DriverController.x().onTrue(elevator.MoveToLevel(LEVELS.FOUR));

    m_DriverController.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    m_DriverController.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    m_DriverController.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    m_DriverController.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

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

    SysIdRoutine sysIdRoutine = shooter.BuildSysIdRoutine();

    m_DriverController.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    m_DriverController.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    m_DriverController.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    m_DriverController.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

    m_DriverController.a().onTrue(shooter.StowShooter());
    m_DriverController.b().onTrue(shooter.MoveToProcess());
    m_DriverController.y().onTrue(shooter.MoveToDeployHigh());
    m_DriverController.x().onTrue(shooter.MoveToDeployLow()); 

    System.out.println("[Wolfpack] Shooter Debug bindings successfully configured.");
  }

  private void configureClimbDebugBindings()
  {
    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

   // climb.setDefaultCommand(climb.setVoltage(() -> -m_DriverController.getRawAxis(5)*10));
  }

  private void configureElevatorShooterDebugBindings()
  {
    Drivetrain drivetrain = m_Manager.getSubsystemOfType(Drivetrain.class).get();

    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    Shooter shooter = m_Manager.getSubsystemOfType(Shooter.class).get();
    SmartDashboard.putData(shooter);

    ShooterCoral shooterCoral = m_Manager.getSubsystemOfType(ShooterCoral.class).get();
    SmartDashboard.putData(shooterCoral);
    
    drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );

    m_DriverController.a().onTrue(new ParallelCommandGroup(shooter.MoveToDeployLow(), elevator.MoveToLevel(LEVELS.TWO)))
                          .onFalse(new ParallelCommandGroup(shooter.StowShooter(), elevator.MoveToLevel(LEVELS.HOME)));

    m_DriverController.b().onTrue(new ParallelCommandGroup(shooter.MoveToDeployLow(), elevator.MoveToLevel(LEVELS.THREE)))
                          .onFalse(new ParallelCommandGroup(shooter.StowShooter(), elevator.MoveToLevel(LEVELS.HOME)));
    
    m_DriverController.y().onTrue(new ParallelCommandGroup(shooter.MoveToDeployHigh(), elevator.MoveToLevel(LEVELS.FOUR)))
                          .onFalse(new ParallelCommandGroup(shooter.StowShooter(), elevator.MoveToLevel(LEVELS.HOME)));

    m_DriverController.rightBumper().onTrue(shooterCoral.DeployCoralRoutine()).onFalse(shooterCoral.StopCoral());

    m_DriverController.leftTrigger().whileTrue(new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(),
                                              elevator.MoveToLevel(LEVELS.CORAL_INTAKE),
                                              shooter.MoveToIntake()));
                                            }

  public Command getAutonomousCommand() 
  {
    return m_Chooser.getSelected();
  }
}
