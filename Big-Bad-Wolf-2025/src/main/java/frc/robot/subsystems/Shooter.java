// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Hardware;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase 
{
  private final TalonFX m_ShooterWristMotor;
  private final TalonFX m_ShooterShooterMotor;

  private SysIdRoutine m_SysIdRoutine;

  /** Creates a new Shooter. */
  public Shooter()
  {
    m_ShooterWristMotor = new TalonFX(Hardware.SHOOTER_WRIST_MOTOR);
    m_ShooterShooterMotor = new TalonFX(Hardware.SHOOTER_CORAL_MOTOR);

    m_ShooterWristMotor.getConfigurator().apply(ShooterConstants.WRIST_MOTOR_CONFIG);
    m_ShooterShooterMotor.getConfigurator().apply(ShooterConstants.WRIST_MOTOR_CONFIG);
  }


  public Command applyWristVoltage(double voltage) //second voltage used to be 0.015 before brake mode
  {
    return this.startEnd(() -> m_ShooterWristMotor.setControl(new VoltageOut(voltage)), () -> m_ShooterWristMotor.setControl(new StaticBrake()));
  }

  public Command applyWristBrake()
  {
    return this.runOnce(() -> m_ShooterWristMotor.setControl(new StaticBrake()));
  }

  public Command applyShooterVoltage(double voltage)
  {
    return this.startEnd(() -> m_ShooterShooterMotor.setControl(new VoltageOut(voltage)), () -> m_ShooterShooterMotor.setControl(new VoltageOut(0)));
  }

  public SysIdRoutine buildSysIdRoutine(TalonFX motor)
  {
    this.m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,         // Use default ramp rate (1 V/s)
        Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
        null,          // Use default timeout (10 s)
        (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> motor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );
    return this.m_SysIdRoutine;
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Shooter Wrist Position",  m_ShooterWristMotor.getPosition().getValueAsDouble());
  }
}
