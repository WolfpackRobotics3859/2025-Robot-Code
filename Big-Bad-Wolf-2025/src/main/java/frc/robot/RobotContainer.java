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
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralPurge;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Global.BUILD_TYPE;
import frc.robot.utilities.SubsystemManager;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  public static final CoralPlacer m_CoralPlacer = new CoralPlacer();
  
  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);
  
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //         .withDeadband(TunerConstants.MaxSpeed * 0.1).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
  //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // this.configurationChooser(Global.ACTIVE_BUILD);
    configurePlacerDebugBindings();
  }

  // public static SubsystemManager getSubsystemManager()
  // {
  //   return m_Manager;
  // }

  // private void configurationChooser(BUILD_TYPE type)
  // {
  //   switch(type)
  //   {
  //     case COMPETITION:
  //       m_Manager.addSubsystem(TunerConstants.createDrivetrain());
  //       m_Manager.addSubsystem(new Elevator());
  //       m_Manager.addSubsystem(new AlgaeIntake());
  //       m_Manager.addSubsystem(new AlgaeCleaner());
  //       m_Manager.addSubsystem(new CoralPlacer());
  //       m_Manager.addSubsystem(new Climb());
  //       this.configureCompetitionBindings();
  //     break;

  //     case DRIVETRAIN_DEBUG:
  //       m_Manager.addSubsystem(TunerConstants.createDrivetrain());
  //       this.configureDrivetrainDebugBindings();
  //     break;

  //     case PLACER_DEBUG:
  //       m_Manager.addSubsystem(new Elevator());
  //       this.configurePlacerDebugBindings();
  //     break;

  //     default:
  //       System.out.println("Did you mean to configure nothing? :( Sad Robot Face");
  //     break;
  //   }
  //   SmartDashboard.putString("Active Build", type.name());
  // }

  // private void configureCompetitionBindings()
  // {
  //   // Emptry for now
  // }

//  private void configureDrivetrainDebugBindings()
//   {
//     CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
//     SmartDashboard.putData((Sendable) drivetrain);

//     m_DriverController.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//     m_DriverController.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//     m_DriverController.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//     m_DriverController.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

//     drivetrain.setDefaultCommand
//     (
//         m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get().applyRequest(() ->
//             drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed)
//                  .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed)
//                  .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate)
//         )
//     );
//     m_DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
//     m_DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  // }

  /*
   * Configures the bindings for characterization subroutines. Shouldn't be bound in a competition environment.
   */
  private void configurePlacerDebugBindings()
  {
    m_DriverController.rightBumper().whileTrue(new CoralPurge(m_CoralPlacer));
    m_DriverController.leftBumper().whileTrue(new CoralIntake(m_CoralPlacer));

    m_DriverController.povDown().whileTrue(m_CoralPlacer.applyPlacerVoltage(-1, m_CoralPlacer.m_CoralPlacerRollerMotor));
    m_DriverController.rightTrigger().whileTrue(m_CoralPlacer.applyPlacerVoltage(1, m_CoralPlacer.m_CoralPlacerWristMotor));
    m_DriverController.povUp().whileTrue(m_CoralPlacer.applyPlacerVoltage(1, m_CoralPlacer.m_CoralPlacerRollerMotor));
    m_DriverController.leftTrigger().whileTrue(m_CoralPlacer.applyPlacerVoltage(-1, m_CoralPlacer.m_CoralPlacerWristMotor));

    
  }

  public Command getAutonomousCommand() 
  {
      return Commands.print("No autonomous command configured");
  }
}
