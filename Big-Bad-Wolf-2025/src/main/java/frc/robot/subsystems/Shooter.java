// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;

public class Shooter extends SubsystemBase 
{
  private final TalonFX m_ShooterWristMotor;
  private final TalonFX m_ShooterAlgaeMotor;
  private final TalonFX m_ShooterCoralMotor;

  private final TimeOfFlight m_TOF;

  private SysIdRoutine m_SysIdRoutine;

  private final VoltageOut m_AlgaeVoltageRequest;
  private final VoltageOut m_CoralVoltageRequest;
  private final MotionMagicVoltage m_WristPositionRequest;
  private final StaticBrake m_BrakeRequest;

  public Shooter()
  {
    MotorManager.AddMotor("SHOOTER WRIST MOTOR", Hardware.SHOOTER_WRIST_MOTOR);
    MotorManager.AddMotor("SHOOTER ALGAE MOTOR", Hardware.SHOOTER_ALGAE_MOTOR);
    MotorManager.AddMotor("SHOOTER CORAL MOTOR", Hardware.SHOOTER_CORAL_MOTOR);

    m_ShooterWristMotor = MotorManager.GetMotor(Hardware.SHOOTER_WRIST_MOTOR);
    m_ShooterAlgaeMotor = MotorManager.GetMotor(Hardware.SHOOTER_ALGAE_MOTOR);
    m_ShooterCoralMotor = MotorManager.GetMotor(Hardware.SHOOTER_CORAL_MOTOR);
    m_TOF = new TimeOfFlight(Hardware.CORAL_TOF_SENSOR);

    MotorManager.ApplyConfigs(ShooterConstants.WRIST_MOTOR_CONFIG, Hardware.SHOOTER_WRIST_MOTOR);
    MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_ALGAE_MOTOR_CONFIG, Hardware.SHOOTER_ALGAE_MOTOR);
    MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_CORAL_MOTOR_CONFIG, Hardware.SHOOTER_CORAL_MOTOR);

    m_AlgaeVoltageRequest = new VoltageOut(0);
    m_CoralVoltageRequest = new VoltageOut(0);
    m_WristPositionRequest = new MotionMagicVoltage(0);
    m_BrakeRequest = new StaticBrake();
  }

  public Command PrepareToDeployCoral()
  {
    return new FunctionalCommand(() -> this.BrakeAlgae().BrakeCoral().SetWristPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION),
                                 () -> {}, 
                                 interrupted -> {},
                                 MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.1),
                                 this);
  }

  public Command DeployCoral()
  {
    return this.runOnce(() -> this.SetCoralVoltage(ShooterConstants.CORAL_DEPLOYMENT_VOLTAGE));
  }

  public Command StowShooter()
  {
    return new FunctionalCommand(() -> this.BrakeAlgae().BrakeCoral().SetWristPosition(ShooterConstants.WRIST_STOW_POSITION),
                                 () -> {}, 
                                 interrupted -> {},
                                 MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.1),
                                 this);
  }

  public Command StowAndHoldAlgae()
  {
    return new FunctionalCommand(() -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_HOLDING_VOLTAGE).BrakeCoral().SetWristPosition(ShooterConstants.WRIST_STOW_POSITION),
                                 () -> {}, 
                                 interrupted -> {},
                                 MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.1),
                                 this);
  }

  public Command IntakeCoral()
  {
    return new FunctionalCommand(() -> this.BrakeAlgae().SetCoralVoltage(ShooterConstants.CORAL_INTAKE_VOLTAGE).SetWristPosition(ShooterConstants.WRIST_CORAL_INTAKE_POSITION),
                                 () -> {}, 
                                 interrupted -> this.BrakeCoral(),
                                 () -> this.CoralDetected(),
                                 this);
  }

  public Command ProcessAlgae()
  {
    return new FunctionalCommand(() -> this.BrakeAlgae().BrakeCoral().SetWristPosition(ShooterConstants.WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION),
                                 () -> {}, 
                                 interrupted -> this.SetAlgaeVoltage(ShooterConstants.ALGAE_PROCESSOR_DEPLOYMENT_VOLTAGE),
                                 MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.005),
                                 this);
  }

  public Command SweepAlgae()
  {
    return SetAlgaeAndWrist(ShooterConstants.ALGAE_SWEEPING_VOLTAGE, ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION);
  }

  public Command IntakeAlgae()
  {
    return SetAlgaeAndWrist(ShooterConstants.ALGAE_GROUND_INTAKING_VOLTAGE, ShooterConstants.WRIST_ALGAE_INTAKE_POSITION);
  }

  public Command SetAlgaeAndWrist(double algaeVoltage, double position)
  {
    return new FunctionalCommand(() -> this.BrakeCoral().SetAlgaeVoltage(algaeVoltage).SetWristPosition(position),
    () -> {}, 
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.5),
    this);
  }

  public Command SetCoralAndWrist(double coralVoltage, double position)
  {
    return new FunctionalCommand(() -> this.SetCoralVoltage(coralVoltage).BrakeAlgae().SetWristPosition(position),
    () -> {}, 
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.5),
    this);
  }

  public Command SetShooterWristPositon(double position)
  {
    return new FunctionalCommand(() -> this.SetWristPosition(position),
                                 () -> {}, 
                                 interrupted -> {},
                                 MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR, 0.5),
                                 this);
  }

  public Command SetAlgaeVoltageInstant(double voltage)
  {
    return this.runOnce(() -> this.SetAlgaeVoltage(voltage));
  }
  
  public Command SetCoralVoltageInstant(double voltage)
  {
    return this.runOnce(() -> this.SetCoralVoltage(voltage));
  }

  // To-do: Move sysId settings to the constants file
  public SysIdRoutine BuildSysIdRoutine()
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.2).per(Seconds),         // Use default ramp rate (1 V/s)
         Volts.of(0.3), // Reduce dynamic step voltage to 4 to prevent brownout
         null,          // Use default timeout (10 s)
         (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_ShooterWristMotor.setControl(new VoltageOut(volts.in(Volts))),
         null,
         this
      )
   );
   return this.m_SysIdRoutine;
  }

  private Shooter SetWristPosition(double position)
  {
    MotorManager.ApplyControlRequest(m_WristPositionRequest.withPosition(position), Hardware.SHOOTER_WRIST_MOTOR);
    return this;
  }

  private Shooter SetAlgaeVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_AlgaeVoltageRequest.withOutput(voltage), Hardware.SHOOTER_ALGAE_MOTOR);
    return this;
  }

  private Shooter SetCoralVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_CoralVoltageRequest.withOutput(voltage), Hardware.SHOOTER_CORAL_MOTOR);
    return this;
  }

  private Shooter BrakeAlgae()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.SHOOTER_ALGAE_MOTOR);
    return this;
  }

  private Shooter BrakeCoral()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.SHOOTER_CORAL_MOTOR);
    return this;
  }

  private boolean CoralDetected()
  {
    return this.m_TOF.getRange() < ShooterConstants.CORAL_TOF_IN_RANGE_THRESHOLD;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("TOF DATA", m_TOF.getRange());
  }
}
