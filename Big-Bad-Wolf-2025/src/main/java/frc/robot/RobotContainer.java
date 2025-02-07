// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgaeIntakeWristUp;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.AlgaeIntakeGroundCommand;
import frc.robot.commands.AlgaeIntakeProcessingCommand;
import frc.robot.constants.AlgaeIntakeConstants;
// import frc.robot.commands.CoralIntake;
// import frc.robot.commands.CoralPurge;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  public static final CoralPlacer m_CoralPlacer = new CoralPlacer();
  public static final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  public static final Climb m_Climb = new Climb();

  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.MaxSpeed * 0.1).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // m_Manager.addSubsystem(TunerConstants.createDrivetrain());
    // this.configureDrivetrainDebugBindings();

    configureAlgaeIntakeDebugBindings();

    configureClimbDebugBindings();

    configurePlacerDebugBindings();


  }

  public static SubsystemManager getSubsystemManager()
  {
    return m_Manager;
  }

  // private void configurationChooser(BUILD_TYPE type)
  // {
  //   switch(type)
  //   {
  //     case COMPETITION:
  //       m_Manager.addSubsystem(TunerConstants.createDrivetrain());
       
  //       //m_Manager.addSubsystem(new CoralPlacer());
  //       this.configureCompetitionBindings();
  //     break;

  //     case DRIVETRAIN_DEBUG:
  //       m_Manager.addSubsystem(TunerConstants.createDrivetrain());
  //       this.configureDrivetrainDebugBindings();
  //     break;

  //     // case PLACER_DEBUG:
  //     //   m_Manager.addSubsystem(new Elevator());
  //     //   this.configurePlacerDebugBindings();
  //     // break;

  //     default:
  //       System.out.println("Did you mean to configure nothing? :( Sad Robot Face");
  //     break;
  //   }
  //   SmartDashboard.putString("Active Build", type.name());
  // }

private void configureCompetitionBindings()
{
  CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
  // Elevator elevator =m_Manager.getSubsystemOfType(Elevator.class).get();
  // Intake intake = m_Manager.getSubsystemOfType(Intake.class).get();
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
  m_DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
  m_DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); //resets drive pose
  //elevator
  // m_DriverController.povUp().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE));
  // m_DriverController.povDown().whileTrue(elevator.applyElevatorVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE));
  // //intake
  // m_DriverController.rightTrigger().whileTrue(intake.goToPositionThenRoll(AlgaeIntakeConstants.ALGAE_INTAKE_WRIST_GROUND_POSITION, -3));
  // //process algae
  // m_DriverController.a().whileTrue(intake.goToPositionThenRoll(6, 3));

}
 private void configureDrivetrainDebugBindings()
  {
    CommandSwerveDrivetrain m_Drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
    SmartDashboard.putData((Sendable) m_Drivetrain);

    m_DriverController.a().whileTrue(m_Drivetrain.sysIdDynamic(Direction.kForward));
    m_DriverController.b().whileTrue(m_Drivetrain.sysIdDynamic(Direction.kReverse));
    m_DriverController.y().whileTrue(m_Drivetrain.sysIdQuasistatic(Direction.kForward));
    m_DriverController.x().whileTrue(m_Drivetrain.sysIdQuasistatic(Direction.kReverse));

    m_Drivetrain.setDefaultCommand
    (
        m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
            drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
                 .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
                 .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
        )
    );
    m_DriverController.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));
    m_DriverController.leftBumper().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldCentric()));
  }

  /*
   * Configures the bindings for characterization subroutines. Shouldn't be bound in a competition environment.
   */
  private void configurePlacerDebugBindings()
  {
    //CoralPlacer m_CoralPlacer = m_Manager.getSubsystemOfType(CoralPlacer.class).get();
    SmartDashboard.putData(m_CoralPlacer);

    // m_DriverController.rightBumper().whileTrue(new CoralPurge(m_CoralPlacer));
    // m_DriverController.leftBumper().whileTrue(new CoralIntake(m_CoralPlacer));

    m_DriverController.leftTrigger().whileTrue(m_CoralPlacer.applyWristVoltage(-1, 0.015));//up
    m_DriverController.povDown().whileTrue(m_CoralPlacer.applyWristVoltage(-1, 0.015));//down
    m_DriverController.rightTrigger().whileTrue(m_CoralPlacer.applyWristVoltage(1, 0));//down
    m_DriverController.y().whileTrue(m_CoralPlacer.applyShooterVoltage(-2));//outake
    m_DriverController.b().whileTrue(m_CoralPlacer.applyShooterVoltage(2));//intake
  }

  private void configureAlgaeIntakeDebugBindings()
  {
    //CoralPlacer m_CoralPlacer = m_Manager.getSubsystemOfType(CoralPlacer.class).get();
    SmartDashboard.putData(m_AlgaeIntake);

    // m_DriverController.rightBumper().whileTrue(new CoralPurge(m_CoralPlacer));
    // m_DriverController.leftBumper().whileTrue(new CoralIntake(m_CoralPlacer));

    m_DriverController.a().whileTrue(new AlgaeIntakeGroundCommand(m_AlgaeIntake));
    // m_DriverController.leftBumper().whileTrue(new AlgaeIntakeWristUp(m_AlgaeIntake));
    m_DriverController.x().whileTrue(new AlgaeIntakeProcessingCommand(m_AlgaeIntake));
  }

  private void configureClimbDebugBindings()
  {
    //CoralPlacer m_CoralPlacer = m_Manager.getSubsystemOfType(CoralPlacer.class).get();
    SmartDashboard.putData(m_Climb);

    // m_DriverController.rightBumper().whileTrue(new CoralPurge(m_CoralPlacer));
    // m_DriverController.leftBumper().whileTrue(new CoralIntake(m_CoralPlacer));

    m_DriverController.rightBumper().whileTrue(new ClimbCommand(m_Climb, 1));
    m_DriverController.leftBumper().whileTrue(new ClimbCommand(m_Climb, -1));
    // m_DriverController.y().whileTrue(new ClimbCommand(m_Climb, 3));
    // m_DriverController.b().whileTrue(new ClimbCommand(m_Climb, 0));
  }

  

  public Command getAutonomousCommand() 
  {
      return Commands.print("No autonomous command configured");
  }
}
