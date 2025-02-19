// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SubsystemManager;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

public class Automation extends SubsystemBase implements SubsystemAddedListener
{
  private SubsystemManager m_Subsystems;
  private CommandSwerveDrivetrain m_Drivetrain;

  private SwerveRequest.ApplyRobotSpeeds m_SwerveRequest;

  public Automation(SubsystemManager manager) 
  {
    this.m_Subsystems = manager;
    if(this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).isPresent())
    {
      this.m_Drivetrain = this.m_Subsystems.getSubsystemOfType(CommandSwerveDrivetrain.class).get();
      this.Configure();
    }
    else
    {
      m_Subsystems.subscribeSubsystemAdded(this);
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void onSubsystemAddedEvent(SubsystemAddedEvent event) 
  {
    if(event.getSubsystem().getClass() == CommandSwerveDrivetrain.class)
    {
      this.m_Drivetrain = (CommandSwerveDrivetrain) event.getSubsystem();
      m_Subsystems.unsubscribeSubsystemAdded(this);
      this.Configure();
    }
  }

  private void Configure()
  {
    this.ConfigureAutobuilder(); // autobuilder should be configured last?
  }

  private void ConfigureAutobuilder()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();
    } 
    catch (Exception e) 
    {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
            () -> this.m_Drivetrain.getState().Pose, // Robot pose supplier
            (pose) -> this.m_Drivetrain.resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            () -> this.m_Drivetrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> this.m_Drivetrain.setControl(m_SwerveRequest.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this.m_Drivetrain
    );
  }
}
