// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class CoralPlacer extends SubsystemBase
{
  private final TalonFX m_CoralPlacerRollerMotor;
  private final TalonFX m_CoralPlacerWristMotor;

  public final TimeOfFlight m_FrontLaser;
  public final TimeOfFlight m_BackLaser;

  public final Timer intakeTimer;

  /**
   * Creates new CoralPlacer subsystem; initializes motors and sensors
   */
  public CoralPlacer()
  {
    m_CoralPlacerRollerMotor = new TalonFX(Hardware.CORAL_PLACER_ROLLER_MOTOR);
    m_CoralPlacerWristMotor = new TalonFX(Hardware.CORAL_PLACER_WRIST_MOTOR);

    m_FrontLaser = new TimeOfFlight(Hardware.CORAL_PLACER_FRONT_LASER);
    m_BackLaser = new TimeOfFlight(Hardware.CORAL_PLACER_BACK_LASER);
    
    intakeTimer = new Timer();
  }

  /** 
   * Boolean for detecting whether an object is within front laser's detected range
   * 
   * @return True if the laser detected range is less than its normal value (if it detects something)
   */
  public boolean frontLaserDetect()
  {
    return this.m_FrontLaser.getRange() < CoralPlacerConstants.FRONT_LASER_LIMIT;
  }

  /** 
   * Boolean for detecting whether an object is within back laser's detected range
   * 
   * @return True if the laser's detected range is less than its normal value (if it detects something)
   */
  public boolean backLaserDetect()
  {
    return this.m_BackLaser.getRange() < CoralPlacerConstants.BACK_LASER_LIMIT;
  }

  /** 
   * Logic for accepting coral from feeder using outputs form both TimeOfFLight sensors.
   */
  public void IntakeCoral()
  {
    if (frontLaserDetect())
    {
      // If front laser detects: reset and stop timer
      intakeTimer.reset();
      if (backLaserDetect())
      {
        // If front laser and back laser detect then there's two corals in intake: purge coral
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, CoralPlacerConstants.CORAL_PLACER_PURGE_VOLTAGE);
      }
      else
      {
        // If front laser detects but back doesn't: brake to hold coral
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.BRAKE, 0);
      }
    }
    else if (backLaserDetect())
    {
      // If back laser detects: start intakeTimer
      intakeTimer.start();

      if (intakeTimer.get() >= CoralPlacerConstants.INTAKE_TIME_THRESHOLD && intakeTimer.get() < CoralPlacerConstants.UNJAM_TIME_THRESHOLD)
      {
        // If back laser detect and timer is greater than or equal to INTAKE_TIME_THRESHOLD and less than UNJAM_TIME_THRESHOLD: unjam
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, CoralPlacerConstants.CORAL_PLACER_UNJAM_VOLTAGE);
      }
      else if (intakeTimer.get() >= CoralPlacerConstants.UNJAM_TIME_THRESHOLD)
      {
        /* If back laser detect and timer greater or equal to UNJAM_TIME_THRESHOLD: intake
        We only reset timer because loop will trigger else if (backLaserDetect) and instantly go to else statement*/
        intakeTimer.reset();
      }
      else
      {
        // If back laser detect and timer less than INTAKE_TIME_THRESHOLD: intake
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, CoralPlacerConstants.CORAL_PLACER_INTAKE_VOLTAGE); 
      }
    }
    else
    {
      if (intakeTimer.get() >= CoralPlacerConstants.INTAKE_TIME_THRESHOLD)
      {
        // If neither lasers detect and timer is over threshold for intaking: stop intake, stop and reset timer
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, 0);
        intakeTimer.reset();
      }
      else
      {
        // If neither lasers detect but timer isn't over threshold for intaking: intake
        CoralPlacerRequest(CoralPlacerConstants.MOTOR.PLACER_ROLLER, Global.MODE.VOLTAGE, CoralPlacerConstants.CORAL_PLACER_INTAKE_VOLTAGE);
      }
    }
  }
  
  /** 
   * Method for controlling any motor in the coral placer subsystem.
   * 
   * @param motor chosen motor to use
   * @param controlMode Mode to set motor input
   * @param value motor input
   */
  public void CoralPlacerRequest(CoralPlacerConstants.MOTOR motor,Global.MODE controlMode, double value)
  {
    switch(controlMode)
    {
      case VOLTAGE:
        setMotorVoltage(motor, value);
        break;
      case PERCENT:
        setMotorPercentage(motor, value);
        break;
      case POSITION:
        setMotorPosition(motor, value);
        break;
      case BRAKE:
        setMotorBrake(motor);
        break;
    }
  }

  /**
   * Sets voltage for chosen motor.
   * 
   * @param motor chosen motor
   * @param voltage amount of volts
   */
  private void setMotorVoltage(CoralPlacerConstants.MOTOR motor, double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    switch (motor) 
    {
      case PLACER_ROLLER:
        m_CoralPlacerRollerMotor.setControl(voltageRequest);
        break;
      case PLACER_WRIST:
        m_CoralPlacerWristMotor.setControl(voltageRequest);
        break;
    }
  }

  /** 
   * Sets chosen motor to work at a percentage of the max power it can.
   * 
   * @param motor chosen motor
   * @param percentage percentage of max power motor works at.
   */
  private void setMotorPercentage(CoralPlacerConstants.MOTOR motor, double percentage)
  {
    DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
    switch (motor) 
    {
      case PLACER_ROLLER:
        m_CoralPlacerRollerMotor.setControl(dutyCycleRequest);
        break;
      case PLACER_WRIST:
        m_CoralPlacerWristMotor.setControl(dutyCycleRequest);
        break;
    }
  }

  /** 
   * Sets chosen motor to move to a position.
   * 
   * @param motor chosen motor
   * @param position percentage of max power motor works at.
   */
  private void setMotorPosition(CoralPlacerConstants.MOTOR motor, double position)
  {
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(position);
    switch (motor) 
    {
      case PLACER_ROLLER:
        System.out.println("WARNING: ROLLER MOTOR DOES NOT USE THIS FUNCTIONALITY");
        break;
      case PLACER_WRIST:
        m_CoralPlacerWristMotor.setControl(positionRequest);
        break;
    }
  }

  /** 
   * Sets chosen motor to resist any external forces
   * 
   * @param motor chosen motor
   */
  private void setMotorBrake(CoralPlacerConstants.MOTOR motor)
  {
    StaticBrake brakeRequest = new StaticBrake();
    switch (motor) 
    {
      case PLACER_ROLLER:
        m_CoralPlacerRollerMotor.setControl(brakeRequest);
        break;
      case PLACER_WRIST:
        m_CoralPlacerWristMotor.setControl(brakeRequest);
        break;
    }
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    // Empty as of now
  }
}
