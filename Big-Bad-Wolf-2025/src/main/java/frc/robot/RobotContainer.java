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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralPlacer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  public static final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.createDrivetrain(); 
  public static final Elevator m_Elevator = new Elevator();
  public static final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
  public static final CoralPlacer m_CoralPlacer = new CoralPlacer();


  private final CommandXboxController m_DriverController = new CommandXboxController(0);
  private final CommandXboxController m_CoDriverController = new CommandXboxController(1);
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.MaxSpeed * 0.1).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public Command forwardDynamicCommand = m_Drivetrain.sysIdDynamic(Direction.kForward);
  public Command reverseDynamicCommand = m_Drivetrain.sysIdDynamic(Direction.kReverse);
  public Command forwardQuasistaticCommand = m_Drivetrain.sysIdQuasistatic(Direction.kForward);
  public Command reverseQuasistaticCommand = m_Drivetrain.sysIdQuasistatic(Direction.kReverse);
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    configureBindings();
    configureCharacterizationBindings(); // REMOVE FROM COMPETITION BUILD (REMOVE_BEFORE_COMP)
  }

  private void configureBindings() 
  {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      m_Drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          m_Drivetrain.applyRequest(() ->
              drive.withVelocityX(-m_DriverController.getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(-m_DriverController.getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-m_DriverController.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );

      m_DriverController.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));
  
      m_DriverController.leftBumper().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldCentric()));
  }

  private void configureCharacterizationBindings()
  {
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
