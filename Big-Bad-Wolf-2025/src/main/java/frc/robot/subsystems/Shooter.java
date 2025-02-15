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

import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.MotorManager;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

// Algae Cleaner subsystem.
public class Shooter extends SubsystemBase 
{
  private final TalonFX m_ShooterWristMotor;
  private final TalonFX m_ShooterCoralMotor;

//   private final TimeOfFlight m_TOF;

  private SysIdRoutine m_SysIdRoutine;

  private final VoltageOut m_CoralVoltageRequest;
  private final MotionMagicVoltage m_WristPositionRequest;
  private final StaticBrake m_BrakeRequest;

  /**
   *  Algae Cleaner subsystem constructor.
   */
  public Shooter() 
  {
    MotorManager.AddMotor("SHOOTER CORAL MOTOR", Hardware.SHOOTER_CORAL_MOTOR_ID);
    MotorManager.AddMotor("SHOOTER WRIST MOTOR", Hardware.SHOOTER_WRIST_MOTOR_ID);

    m_ShooterWristMotor = MotorManager.GetMotor(Hardware.SHOOTER_WRIST_MOTOR_ID);
    m_ShooterCoralMotor = MotorManager.GetMotor(Hardware.SHOOTER_CORAL_MOTOR_ID);

    MotorManager.ApplyConfigs(ShooterConstants.WRIST_MOTOR_CONFIG, Hardware.SHOOTER_WRIST_MOTOR_ID);
    MotorManager.ApplyConfigs(ShooterConstants.SHOOTER_CORAL_MOTOR_CONFIG, Hardware.SHOOTER_CORAL_MOTOR_ID);

    m_CoralVoltageRequest = new VoltageOut(0);
    m_WristPositionRequest = new MotionMagicVoltage(0);
    m_BrakeRequest = new StaticBrake();
  }

  public Command IntakeCoral()
  {
    return SetCoralWrist(ShooterConstants.CORAL_INTAKING_ROLLER_VOLTAGE, ShooterConstants.WRIST_CORAL_INTAKE_POSITION);
  }

  public Command SetCoralWrist(double coralVoltage, double position)
  {
    return new FunctionalCommand(() -> this.SetCoralVoltage(coralVoltage).BrakeAlgae().SetWristPosition(position),
    () -> {},
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR_ID, 0.5),
    this);
  }

  public Command SetLevelTwoCoralPlacer(double coralVoltage, double position)
  {
    return new FunctionalCommand(() -> this.SetCoralVoltage(coralVoltage).BrakeAlgae().SetWristPosition(position),
    () -> {},
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR_ID, 0.5),
    this);
  }

  public Command SetLevelThreeCoralPlacer(double coralVoltage, double position)
  {
    return new FunctionalCommand(() -> this.SetCoralVoltage(coralVoltage).BrakeAlgae().SetWristPosition(position),
    () -> {},
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR_ID, 0.5),
    this);
  }

  public Command SetLevelFourCoralPlacer(double coralVoltage, double position)
  {
    return new FunctionalCommand(() -> this.SetCoralVoltage(coralVoltage).BrakeAlgae().SetWristPosition(position),
    () -> {},
    interrupted -> {},
    MotorManager.InPosition(Hardware.SHOOTER_WRIST_MOTOR_ID, 0.5),
    this);
  }

  private Shooter SetCoralVoltage(double voltage)
  {
    MotorManager.ApplyControlRequest(m_CoralVoltageRequest.withOutput(voltage), Hardware.SHOOTER_CORAL_MOTOR_ID);
    return this;
  }

  private Shooter SetWristPosition(double position)
  {
    MotorManager.ApplyControlRequest(m_WristPositionRequest.withPosition(position), Hardware.SHOOTER_ALGAE_MOTOR_ID);
    return this;
  }

  private Shooter BrakeAlgae()
  {
    MotorManager.ApplyControlRequest(m_BrakeRequest, Hardware.SHOOTER_ALGAE_MOTOR_ID);
    return this;
  }

//   /**
//    * Sets the shooting voltage for the Shooter motor.
//    * @param voltage Assigns voltage to the motors (amount of applicable torque).
//    */
//   public void setShooterRollerVoltage(double voltage)
//   {
//     m_ShooterMotor.setControl(new VoltageOut(voltage));
//   }

//   /**
//    * Sets the wrist position for the Algae Cleaner motor.
//    * @param cleanerPosition Assigns wrist position to the motors (where cleaner arm is located in space).
//    */
//   public void setShooterWristPosition(double position)
//   {
//     m_ShooterWristMotor.setControl(new MotionMagicVoltage(position));
//   }

  // SysID routine
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

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
