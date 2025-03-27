// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Global;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants.HEIGHTS;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.constants.WristConstants.ANGLES;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wheels;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.PackLog;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.fieldCalibration.FieldCalibrator;
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
  private SendableChooser<Command> m_Chooser;
  
  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);

  private final PackLog m_PackLog;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    this.m_PackLog = new PackLog("RobotContainer");
    this.configurationChooser(Global.ACTIVE_BUILD);
  }

  private void configurationChooser(BUILD_TYPE type)
  {
    this.m_PackLog.Log("Beginning configuration.");
    switch(type)
    {
      case COMPETITION:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        m_Manager.addSubsystem(new Wheels());
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Climb());
        m_Manager.addSubsystem(new Wrist());
        m_Manager.addSubsystem(new DataStuff());
        this.configureCompetitionBindings();
      break;

      case FIELD_CALIBRATION:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        this.configureFieldCalibrationBindings();
        break;

      case DRIVETRAIN_DEBUG:
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        this.configureDrivetrainDebugBindings();
      break;

      case ELEVATOR_DEBUG:
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(TunerConstants.createDrivetrain());
        this.configureElevatorDebugBindings();
      break;

      case SHOOTER_DEBUG:
        m_Manager.addSubsystem(new Wrist());
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
        m_Manager.addSubsystem(new Wrist());
        m_Manager.addSubsystem(new Wheels());
        m_Manager.addSubsystem(new DataStuff());
        this.configureElevatorShooterDebugBindings();
      break;
      
      default:
        System.out.println("Did you mean to configure nothing? :( Sad Robot Face");
      break;
    }

    this.m_PackLog.Log("Configuration Complete for ACTIVE BUILD: " + type.name());
    SmartDashboard.putString("Active Build", type.name());
  }

  private void configureCompetitionBindings()
  {
    Drivetrain drivetrain = m_Manager.getSubsystemOfType(Drivetrain.class).get();

    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    Wrist wrist = m_Manager.getSubsystemOfType(Wrist.class).get();
    SmartDashboard.putData(wrist);

    Climb climb = m_Manager.getSubsystemOfType(Climb.class).get();
    SmartDashboard.putData(climb);

    DataStuff dataStuff = m_Manager.getSubsystemOfType(DataStuff.class).get();
    SmartDashboard.putData(dataStuff);

    Wheels wheels = m_Manager.getSubsystemOfType(Wheels.class).get();
    SmartDashboard.putData(wheels);

    drivetrain.setDefaultCommand
    (
        drivetrain.DefaultDrive(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX(), () -> m_DriverController.getRightX())
    );

   // Command defaultSafe = new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL));
   // Command alignCoral = new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel());
   // Command beginIntaking = new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.INTAKE)).withDeadline(wheels.BeginCoralIntakeRoutine());
   // Command intakeRoutine = new SequentialCommandGroup(beginIntaking, new ParallelCommandGroup(defaultSafe, wheels.CenterCoral()));

    m_DriverController.rightTrigger().onTrue(new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()).andThen(wheels.DeployCoralLow())) // change later to selection
                                     .onFalse(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.leftTrigger().onTrue(new SequentialCommandGroup(elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.INTAKE)).withDeadline(new SequentialCommandGroup(wheels.BeginCoralIntakeRoutine(), wheels.CenterCoral())))
                                    .onFalse(new ParallelCommandGroup(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)), wheels.CenterCoral()));
    
    m_CoDriverController.leftBumper().onTrue(dataStuff.Left().ignoringDisable(true));
    m_CoDriverController.rightBumper().onTrue(dataStuff.Right().ignoringDisable(true));
    m_CoDriverController.povUp().onTrue(dataStuff.Up().ignoringDisable(true));
    m_CoDriverController.povDown().onTrue(dataStuff.Down().ignoringDisable(true));

    m_CoDriverController.rightTrigger().onTrue(climb.setClimbVoltage(ClimbConstants.CLIMB_WRIST_VOLTAGE)).onFalse(climb.setClimbVoltage(0));

    m_CoDriverController.leftTrigger().onTrue(climb.setRollerVoltage(12)).onFalse(climb.setRollerVoltage(0));

    m_CoDriverController.a().onTrue(climb.setLatchVoltage(-1)).onFalse(climb.setLatchVoltage(0));
    m_CoDriverController.b().onTrue(climb.setLatchVoltage(1)).onFalse(climb.setLatchVoltage(0));
    m_CoDriverController.y().onTrue(climb.setClimbVoltage(-1)).onFalse(climb.setClimbVoltage(0));

    // NamedCommands.registerCommand("ElevatorHome", elevator.MoveToLevel(HEIGHTS.HOME));
    // NamedCommands.registerCommand("CoralTwoDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.TWO), shooter.MoveToDeployLow()));
    // NamedCommands.registerCommand("CoralThreeDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.THREE), shooter.MoveToDeployLow()));
    // NamedCommands.registerCommand("CoralFourDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.FOUR), shooter.MoveToDeployHigh()));
    // NamedCommands.registerCommand("DeployCoral", shooterCoral.DeployCoralRoutine());

    // NamedCommands.registerCommand("Align1L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 1)));
    // NamedCommands.registerCommand("Align1R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 1)));
    // NamedCommands.registerCommand("Align2L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 2)));
    // NamedCommands.registerCommand("Align2R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 2)));
    // NamedCommands.registerCommand("Align3L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 3)));
    // NamedCommands.registerCommand("Align3R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 3)));
    // NamedCommands.registerCommand("Align4L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 4)));
    // NamedCommands.registerCommand("Align4R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 4)));
    // NamedCommands.registerCommand("Align5L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 5)));
    // NamedCommands.registerCommand("Align5R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 5)));
    // NamedCommands.registerCommand("Align6L", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(0, 6)));
    // NamedCommands.registerCommand("Align6R", new ParallelDeadlineGroup(new WaitCommand(1.0), drivetrain.AlignToFace(1, 6)));

    // NamedCommands.registerCommand("ShooterLowPosition", new ParallelCommandGroup(shooter.MoveToDeployLow(), elevator.MoveToLevel(HEIGHTS.ZERO)));

    // NamedCommands.registerCommand("StartIntake", new ParallelDeadlineGroup(shooterCoral.IntakeCoralRoutine(), elevator.MoveToLevel(HEIGHTS.CORAL_INTAKE), shooter.MoveToIntake()));
    
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

  /**
   * Method is used to get exact coordinates for different targets on the field. Align robot with spot you want to remember in the correct orientation, and press 
   * "A" on the driver controller. This prints out the current X, Y, and rotation of the bot for the user to note for getting accurate position data.
   */
  private void configureFieldCalibrationBindings()
  {
    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    FieldCalibrator fieldCalibrator = new FieldCalibrator(drivetrain);

    m_DriverController.a().onTrue(new InstantCommand(() -> 
    {
      System.out.printf("Current X: %f, Current Y: %f, Current Rotation: %s \n",fieldCalibrator.getPose2d().getX(), fieldCalibrator.getPose2d().getY(), fieldCalibrator.getPose2d().getRotation());
    }).ignoringDisable(true));


    m_DriverController.b().onTrue(new InstantCommand(() -> 
    {
        fieldCalibrator.calibratePosition();
    }).ignoringDisable(true));

    m_DriverController.y().onTrue(new InstantCommand(() ->
    {
      ObjectMapper mapper = new ObjectMapper();
      try {
        mapper.writeValue(new File("/home/lvuser/FieldCalibrationConstants.json"), fieldCalibrator);
      } catch (Exception e) {
        e.printStackTrace();
      }

      fieldCalibrator.generateConstantsFile(new File("/home/lvuser/FieldCalibrationConstants.java"));
    }).ignoringDisable(true));
  }


  private void configureDrivetrainDebugBindings()
  {
    Drivetrain drivetrain = m_Manager.getSubsystemOfType(Drivetrain.class).get();

    drivetrain.setDefaultCommand
    (
        drivetrain.DefaultDrive(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX(), () -> m_DriverController.getRightX())
    );

    // m_DriverController.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // m_DriverController.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // m_DriverController.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_DriverController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // drivetrain.setDefaultCommand
    // (
    //     m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
    //         drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
    //              .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
    //              .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
    //     )
    // );
     m_DriverController.x().onTrue(drivetrain.Align(new Pose2d(14.015, 5.1, Rotation2d.fromDegrees(-119.34))));
     m_DriverController.a().whileTrue(drivetrain.AlignXTesting(new Pose2d(1, 1, Rotation2d.k180deg)));
     m_DriverController.b().whileTrue(drivetrain.AlignYTesting(new Pose2d(1, 1, Rotation2d.k180deg)));
     m_DriverController.y().whileTrue(drivetrain.RotationTesting(new Pose2d(1, 1, Rotation2d.k180deg)));
    // m_DriverController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  private void configureElevatorDebugBindings()
  {
    Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
    SmartDashboard.putData(elevator);

    CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

    SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

    m_DriverController.a().onTrue(elevator.MoveToLevel(HEIGHTS.ONE));
    m_DriverController.b().onTrue(elevator.MoveToLevel(HEIGHTS.TWO));
    m_DriverController.y().onTrue(elevator.MoveToLevel(HEIGHTS.THREE));
    m_DriverController.x().onTrue(elevator.MoveToLevel(HEIGHTS.FOUR));

    m_DriverController.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    m_DriverController.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    m_DriverController.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    m_DriverController.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

    System.out.println("[Wolfpack] Elevator Debug bindings successfully configured.");
  }

  private void configureShooterDebugBindings()
  {
    Wrist shooter = m_Manager.getSubsystemOfType(Wrist.class).get();
    SmartDashboard.putData(shooter);

    SysIdRoutine sysIdRoutine = shooter.BuildSysIdRoutine();

    m_DriverController.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    m_DriverController.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    m_DriverController.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    m_DriverController.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

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

    Wrist wrist = m_Manager.getSubsystemOfType(Wrist.class).get();
    SmartDashboard.putData(wrist);

    DataStuff dataStuff = m_Manager.getSubsystemOfType(DataStuff.class).get();
    SmartDashboard.putData(dataStuff);

    Wheels wheels = m_Manager.getSubsystemOfType(Wheels.class).get();
    SmartDashboard.putData(wheels);

    drivetrain.setDefaultCommand
    (
        drivetrain.DefaultDrive(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX(), () -> m_DriverController.getRightX())
    );

    m_DriverController.rightTrigger().whileTrue(new ParallelCommandGroup(wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()))
                                     .onFalse(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.leftTrigger().onTrue(new SequentialCommandGroup(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.INTAKE)).withDeadline(wheels.BeginCoralIntakeRoutine()), new ParallelCommandGroup(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)), wheels.CenterCoral())))
                                    .onFalse(new ParallelCommandGroup(new SequentialCommandGroup(wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)), wheels.CenterCoral()));

    m_DriverController.a().whileTrue(wheels.DeployCoralLow());
    m_DriverController.b().whileTrue(wheels.DeployCoralHigh());

    m_CoDriverController.leftBumper().onTrue(dataStuff.Left().ignoringDisable(true));
    m_CoDriverController.rightBumper().onTrue(dataStuff.Right().ignoringDisable(true));
    m_CoDriverController.povUp().onTrue(dataStuff.Up().ignoringDisable(true));
    m_CoDriverController.povDown().onTrue(dataStuff.Down().ignoringDisable(true));
  }

  public Command getAutonomousCommand() 
  {
    return m_Chooser.getSelected();
  }
}
