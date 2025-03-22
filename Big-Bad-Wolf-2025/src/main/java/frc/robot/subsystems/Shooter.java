// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants.LEVELS;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.DataStuff;
import frc.robot.utilities.MotorManager;

public class Shooter extends SubsystemBase 
{
  private final TalonFX m_ShooterWristMotor;
  private SysIdRoutine m_SysIdRoutine;
  private final MotionMagicVoltage m_WristPositionRequest;
  private final StaticBrake m_BrakeRequest;
  private final CoastOut m_CoastRequest;

  private final StatusSignal<AngularVelocity> m_WristRPS;
  private final StatusSignal<Angle> m_WristPosition;

  public Shooter()
  {
    MotorManager.AddMotor("SHOOTER WRIST MOTOR", Hardware.SHOOTER_WRIST_MOTOR);
    this.m_ShooterWristMotor = MotorManager.GetMotor(Hardware.SHOOTER_WRIST_MOTOR);
    MotorManager.ApplyConfigs(ShooterConstants.WRIST_MOTOR_CONFIG, Hardware.SHOOTER_WRIST_MOTOR);

    this.m_WristPositionRequest = new MotionMagicVoltage(0);
    this.m_BrakeRequest = new StaticBrake();
    this.m_CoastRequest = new CoastOut();

    this.m_WristRPS = this.m_ShooterWristMotor.getVelocity();
    this.m_WristPosition = this.m_ShooterWristMotor.getPosition();

    this.BuildToolbox();
  }

  public Command MoveToSelectedShot()
  {
    return this.runOnce(() -> 
    {
      LEVELS level = DataStuff.GetLevel();
      if(level == LEVELS.FOUR)
      {
        this.ApplyPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION);
      }
      else
      {
        this.ApplyPosition(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION_LOW);
      }
    });
  }

  public Command MoveToSelectorAlgaeDeploy()
  {
    return this.runOnce(() -> 
    {
      double level = DataStuff.GetCurrentLevel();
      if(level < 2)
      {
        this.ApplyPosition(ShooterConstants.WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION);
      }
      else
      {
        this.ApplyPosition(ShooterConstants.WRIST_ALGAE_BARGE_POSITION);
      }
    });
  }


  public Command StowShooter()
  {
    return this.runOnce(() -> ApplyPosition(ShooterConstants.WRIST_STOW_POSITION));
  }

  public Command MoveToBarge()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_BARGE_POSITION);
  }

  public Command MoveToDeployLow()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_CORAL_DEPLOYMENT_POSITION_LOW);
  }

  public Command MoveToIntake()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_CORAL_INTAKE_POSITION);
  }

  public Command MoveToProcess()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION);
  }

  public Command MoveToAlgaeClean()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_INTAKE_POSITION);
  }

  public Command MoveToAlgaeHold()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_HOLDING_POSITION);
  }

  public Command MoveToAlgaeSweep()
  {
    return MoveToCommandBuilder(ShooterConstants.WRIST_ALGAE_SWEEPING_POSITION);
  }

  public Command GoToManualPosition()
  {
    SmartDashboard.putNumber("Manual Shooter Position", 0.0);
    return this.runOnce(() -> this.ApplySmartDashboardPosition());
  }

  public Command SetBrake()
  {
    return this.runOnce(() -> this.ApplyBrake());
  }

  public Command SetCoast()
  {
    return this.runOnce(() -> this.ApplyCoast());
  }

  private Shooter ApplySmartDashboardPosition()
  {
    this.ApplyPosition(SmartDashboard.getNumber("Manual Shooter Position", 0));
    return this;
  }

  private Command MoveToCommandBuilder(double position)
  {
    return new FunctionalCommand(() -> this.ApplyPosition(position),
                                 () -> {}, 
                                 interrupted -> {},
                                 () -> true,
                   //              () -> this.isReady(ShooterConstants.WRIST_POSITION_ERROR_TOLERANCE, ShooterConstants.WRIST_POSITION_DERIVATIVE_TOLERANCE),
                                 this);
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

  private Shooter ApplyPosition(double position)
  {
    MotorManager.ApplyControlRequest(this.m_WristPositionRequest.withPosition(position), Hardware.SHOOTER_WRIST_MOTOR);
    return this;
  }

  private Shooter ApplyBrake()
  {
    MotorManager.ApplyControlRequest(this.m_BrakeRequest, Hardware.SHOOTER_WRIST_MOTOR);
    return this;
  }

  private Shooter ApplyCoast()
  {
    MotorManager.ApplyControlRequest(this.m_CoastRequest, Hardware.SHOOTER_WRIST_MOTOR);
    return this;
  }

  private boolean isInPosition(double tolerance)
  {
    return Math.abs(this.m_ShooterWristMotor.getPosition().getValueAsDouble() - this.m_WristPositionRequest.Position) < tolerance;
  }

  private boolean isReady(double positionTolerance, double derivativeTolerance)
  {
    StatusSignal.refreshAll(this.m_WristRPS, this.m_WristPosition);
    if((Math.abs(this.m_WristPosition.getValueAsDouble()) > derivativeTolerance) || (Math.abs(this.m_WristPosition.getValueAsDouble() - this.m_WristPositionRequest.Position) > positionTolerance))
    {
      return false;
    }
    DataLogManager.log("shooter ready.");

    return true;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("SHOOTER POSITION", this.m_ShooterWristMotor.getPosition().getValueAsDouble());
  }

  private void BuildToolbox()
  {
    SmartDashboard.putData("Static Brake Shooter", this.SetBrake().ignoringDisable(true));
    SmartDashboard.putData("Coast Shooter", this.SetCoast().ignoringDisable(true));
  }
}
