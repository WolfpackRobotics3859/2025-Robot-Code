// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Global;
import frc.robot.constants.WristConstants;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants.HEIGHTS;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.constants.WheelConstants.VoltageSpeeds;
import frc.robot.constants.WristConstants.ANGLES;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Latches;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Lights.LIGHT_CODES;
import frc.robot.subsystems.Wheels;
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
        m_Manager.addSubsystem(new Lights());
        m_Manager.addSubsystem(new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        m_Manager.addSubsystem(new Wheels());
        m_Manager.addSubsystem(new Elevator());
        m_Manager.addSubsystem(new Climb());
        m_Manager.addSubsystem(new Wrist());
        m_Manager.addSubsystem(new DataStuff());
        m_Manager.addSubsystem(new Latches());
        this.configureCompetitionBindings();
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

  public Optional<Lights> GetLights()
  {
    return m_Manager.getSubsystemOfType(Lights.class);
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

    Latches latches = m_Manager.getSubsystemOfType(Latches.class).get();
    SmartDashboard.putData(latches);

    Lights lights = m_Manager.getSubsystemOfType(Lights.class).get();
    SmartDashboard.putData(lights);

    Supplier<Command> flashingGreen = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.FLASHING_GREEN));
    Supplier<Command> flashingRed = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.FLASHING_RED));
    Supplier<Command> idle = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.FADING_ORANGE_AND_BLUE));
    Supplier<Command> flashingOrange = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.FLASHING_ORANGE));
    Supplier<Command> intakeFlags = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.CENTER_INTAKE_FLASH));
    Supplier<Command> weeWoo = () -> new InstantCommand(() -> lights.LightChooser(LIGHT_CODES.FLASH_ALTERNATE_ORANGE_AND_BLUE));

    drivetrain.setDefaultCommand
    (
        drivetrain.DefaultDrive(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX(), () -> m_DriverController.getRightX())
    );

    // // NO PATHFINDING
    // m_DriverController.rightTrigger().whileTrue(new SequentialCommandGroup(flashingRed.get(), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()))
    // .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    // // PATHFIND DIRECT TO CORAL NO ALIGNMENT
    // m_DriverController.rightTrigger().whileTrue(new SequentialCommandGroup(flashingRed.get(), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()))
    // .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));
    
    // // PATHFIND TO STAGING THEN PATHFIND TO CORAL
    // m_DriverController.rightTrigger().whileTrue(new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToStagingUniversal(4, 3, 1), new ParallelCommandGroup(drivetrain.PathfindToCoral(1, 0.5, 0), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()))
    //     .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));
    
    // PATHFIND TO STAGING THEN PID ALIGN
    m_DriverController.rightTrigger().whileTrue(new SequentialCommandGroup(flashingRed.get(), drivetrain.PathfindToStagingUniversal(4, 3, 1), new ParallelCommandGroup(drivetrain.AlignCoral(), wrist.MoveToSelectedCoralDeployment(), elevator.MoveToDataLevel()), wheels.DeployCoral(), flashingGreen.get()))
        .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.leftTrigger().whileTrue(new SequentialCommandGroup(intakeFlags.get(), new ParallelDeadlineGroup(wheels.BeginCoralIntakeRoutine(), elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.INTAKE)), flashingGreen.get()))
                                    .onFalse(new SequentialCommandGroup(flashingOrange.get(), wheels.CenterCoral(), flashingGreen.get(), new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.TRAVEL), wrist.MoveToAngle(ANGLES.TRAVEL)), idle.get()));

    m_DriverController.leftBumper().whileTrue(new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), Commands.waitSeconds(0.1), new ParallelDeadlineGroup(wheels.IntakeAlgae(), drivetrain.AlignCenter(), elevator.MoveToDataClean(), wrist.MoveToAngle(ANGLES.SWEEP)), flashingGreen.get()))
                                   .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));
                        
    m_DriverController.a().whileTrue(new SequentialCommandGroup(flashingRed.get(), wheels.ApplyVoltage(VoltageSpeeds.SWEEP), new WaitCommand(0.1), new ParallelDeadlineGroup(wheels.IntakeAlgae(), elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.GRAB)), flashingGreen.get()))
                          .onFalse(new SequentialCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.b().whileTrue(new SequentialCommandGroup(flashingOrange.get(), new PathPlannerAuto("ProcessRoutine"), flashingGreen.get()))
                          .onFalse(new ParallelCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.povLeft().whileTrue(new SequentialCommandGroup(flashingOrange.get(), new PathPlannerAuto("LeftBargeRoutine"), flashingGreen.get()))
                          .onFalse(new ParallelCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    m_DriverController.povUp().whileTrue(new SequentialCommandGroup(flashingOrange.get(), new PathPlannerAuto("MiddleBargeRoutine"), flashingGreen.get()))
                          .onFalse(new ParallelCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));
                  
    m_DriverController.povRight().whileTrue(new SequentialCommandGroup(flashingOrange.get(), new PathPlannerAuto("RightBargeRoutine"), flashingGreen.get()))
                          .onFalse(new ParallelCommandGroup(idle.get(), wrist.MoveToAngle(ANGLES.TRAVEL), elevator.MoveToLevel(HEIGHTS.TRAVEL)));

    
    m_CoDriverController.leftBumper().onTrue(dataStuff.Left().ignoringDisable(true));
    m_CoDriverController.rightBumper().onTrue(dataStuff.Right().ignoringDisable(true));
    m_CoDriverController.povUp().onTrue(dataStuff.Up().ignoringDisable(true));
    m_CoDriverController.povDown().onTrue(dataStuff.Down().ignoringDisable(true));

    m_CoDriverController.leftStick().onTrue(latches.setLatchVoltage(ClimbConstants.FUNNEL_RELEASE_VOLTAGE)).onFalse(latches.setLatchVoltage(0));
    m_CoDriverController.rightStick().onTrue(latches.setLatchVoltage(ClimbConstants.FOOT_RELEASE_VOTLAGE)).onFalse(latches.setLatchVoltage(0));

    m_CoDriverController.rightTrigger().onTrue(new ParallelCommandGroup(drivetrain.RearDriveSnap(() -> m_DriverController.getLeftY(), () -> m_DriverController.getLeftX()) ,elevator.MoveToLevel(HEIGHTS.ONE), wrist.MoveToAngle(ANGLES.TRAVEL), climb.GoWristPosition(ClimbConstants.CLIMB_TAKING_POSITION)).andThen(climb.ApplyRollerVoltage(ClimbConstants.CLIMB_ROLLER_VOLTAGE)))
                                       .onFalse(climb.ApplyRollerVoltage(0));

    m_CoDriverController.a().onTrue(climb.GoWristPosition(ClimbConstants.CLIMB_RESTING_POSITION));
    m_CoDriverController.b().onTrue(climb.GoWristPosition(ClimbConstants.CLIMB_TAKING_POSITION));
    m_CoDriverController.y().onTrue(climb.GoWristPosition(ClimbConstants.CLIMB_CLIMB_POSITION));
    m_CoDriverController.x().onTrue(climb.setClimbVoltage(0));

    climb.setDefaultCommand(climb.DefaultSafety());

    NamedCommands.registerCommand("FlashingGreen", flashingGreen.get());
    NamedCommands.registerCommand("IntakeFlag", intakeFlags.get());
    NamedCommands.registerCommand("FlashingOrange", flashingOrange.get());
    NamedCommands.registerCommand("WeeWoo", weeWoo.get());

    NamedCommands.registerCommand("ElevatorTravel", elevator.MoveToLevel(HEIGHTS.TRAVEL));
    NamedCommands.registerCommand("ElevatorIntake", elevator.MoveToLevel(HEIGHTS.INTAKE));
    NamedCommands.registerCommand("CoralTwoDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.TWO), wrist.MoveToAngle(ANGLES.DEPLOY_LOW)));
    NamedCommands.registerCommand("CoralThreeDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.THREE), wrist.MoveToAngle(ANGLES.DEPLOY_LOW)));
    NamedCommands.registerCommand("CoralFourDeploy", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.FOUR), wrist.MoveToAngle(ANGLES.DEPLOY_HIGH)));
    NamedCommands.registerCommand("DeployCoralHigh", wheels.DeployCoralHigh());
    NamedCommands.registerCommand("DeployCoralLow", wheels.DeployCoralLow());
    NamedCommands.registerCommand("AlgaeBarge", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.BARGE), wrist.MoveToAngle(ANGLES.BARGE)));
    NamedCommands.registerCommand("AlgaeProcessor", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.GRAB)));
    NamedCommands.registerCommand("DeployAlgae", wheels.DeployAlgae().withTimeout(0.5));
    NamedCommands.registerCommand("Stow", new ParallelCommandGroup(elevator.MoveToLevel(HEIGHTS.TRAVEL), wrist.MoveToAngle(ANGLES.TRAVEL)));

    NamedCommands.registerCommand("Align1L", drivetrain.AlignToFace(0, 1));
    NamedCommands.registerCommand("Align1R", drivetrain.AlignToFace(1, 1));
    NamedCommands.registerCommand("Align2L", drivetrain.AlignToFace(0, 2));
    NamedCommands.registerCommand("Align2R", drivetrain.AlignToFace(1, 2));
    NamedCommands.registerCommand("Align3L", drivetrain.AlignToFace(0, 3));
    NamedCommands.registerCommand("Align3R", drivetrain.AlignToFace(1, 3));
    NamedCommands.registerCommand("Align4L", drivetrain.AlignToFace(0, 4));
    NamedCommands.registerCommand("Align4R", drivetrain.AlignToFace(1, 4));
    NamedCommands.registerCommand("Align5L", drivetrain.AlignToFace(0, 5));
    NamedCommands.registerCommand("Align5R", drivetrain.AlignToFace(1, 5));
    NamedCommands.registerCommand("Align6L", drivetrain.AlignToFace(0, 6));
    NamedCommands.registerCommand("Align6R", drivetrain.AlignToFace(1, 6));

    NamedCommands.registerCommand("StartIntake", new ParallelDeadlineGroup(wheels.BeginCoralIntakeRoutine(), intakeFlags.get(), elevator.MoveToLevel(HEIGHTS.INTAKE), wrist.MoveToAngle(ANGLES.INTAKE)));
    NamedCommands.registerCommand("CenterCoral", wheels.CenterCoral());
    
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
