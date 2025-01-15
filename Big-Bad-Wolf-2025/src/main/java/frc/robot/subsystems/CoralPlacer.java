// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;

public class CoralPlacer extends SubsystemBase
{
  private final TalonFX m_CoralPlacerMotor;

  private final TalonFX shooterWrist = new TalonFX(Hardware.WRIST_MOTOR_ID);
  private final TalonFX shooterRollerTop = new TalonFX(Hardware.SHOOTER_MOTOR_1_ID);
  private final TalonFX shooterRollerBottom = new TalonFX(Hardware.SHOOTER_MOTOR_2_ID);
  private final TalonFX feederRoller = new TalonFX(Hardware.FEEDER_MOTOR_ID);

  public final TimeOfFlight m_BackLaser;
  /**Constructor: When a Coral placer is instantiated everything in the constructor runs
   * currently no parameters
  */
  public CoralPlacer()
  {
    m_CoralPlacerMotor = new TalonFX(Hardware.CORAL_PLACER_MOTOR);
    m_BackLaser = new TimeOfFlight(0);

    shooterWrist.getConfigurator().apply(CoralPlacerConstants.WRIST_MOTOR_CONFIGURATION);
    shooterRollerTop.getConfigurator().apply(CoralPlacerConstants.SHOOTER_MOTOR_1_CONFIGURATION);
    shooterRollerBottom.getConfigurator().apply(CoralPlacerConstants.SHOOTER_MOTOR_2_CONFIGURATION);
  }

  public boolean CoralInPlace()
  {
    return this.m_BackLaser.getRange() > CoralPlacerConstants.BACK_LASER_LIMIT;
  }
  
  public void setMotor(CoralPlacerConstants.MOTOR motor, Global.MODE controlMode, double value){
    switch(controlMode)
    {
      case VOLTAGE:
        this.setMotorVoltage(motor, value);
        break;
      case VELOCITY:
        this.setMotorVelocity(motor, value);
        break;
      case POSITION:
        this.setMotorPosition(motor, value);
        break;
      case PERCENT:
        this.setMotorPercent(motor, value);
        break;
      default:
        System.out.println("WARNING: That is not a mood (mode).");
        break;
    }
  }
  
//select motor in correspondance to previously selected mode above----------------
  private void setMotorVoltage(CoralPlacerConstants.MOTOR motor, double voltage){
    VoltageOut voltageRequest = new VoltageOut(voltage);
    switch(motor){

        case SHOOTER_MOTOR_1:
          shooterRollerTop.setControl(voltageRequest);
          break;

        case SHOOTER_MOTOR_2:
          shooterRollerBottom.setControl(voltageRequest);
          break;

        case WRIST_MOTOR:
          shooterWrist.setControl(voltageRequest);
          break;

        case FEEDER_MOTOR:
          feederRoller.setControl(voltageRequest);
          break;
            
        default:
          System.out.println("WARNING: That motor doesn't exist!");
          break;
      }
  }

  private void setMotorVelocity(CoralPlacerConstants.MOTOR motor, double velocity){
    MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(velocity);
    switch(motor){

        case SHOOTER_MOTOR_1:
        shooterRollerTop.setControl(velocityRequest);
          break;

        case SHOOTER_MOTOR_2:
          shooterRollerBottom.setControl(velocityRequest);
          break;

        case WRIST_MOTOR:
          System.out.println("WARNING: don't put velocity on the wrist!");
          break;

        case FEEDER_MOTOR:
          feederRoller.setControl(velocityRequest);
          break;
            
        default:
          System.out.println("WARNING: That motor doesn't exist!");
          break;
    }

  }

  private void setMotorPosition(CoralPlacerConstants.MOTOR motor, double position){
  MotionMagicVoltage positionRequest = new MotionMagicVoltage(position);
      switch(motor){

          case SHOOTER_MOTOR_1:
            shooterRollerTop.setControl(positionRequest);
            break;

          case SHOOTER_MOTOR_2:
            shooterRollerBottom.setControl(positionRequest);
            break;

          case WRIST_MOTOR:
            positionRequest = new MotionMagicVoltage(position);
            shooterWrist.setControl(positionRequest);
            break;

          case FEEDER_MOTOR:
            feederRoller.setControl(positionRequest);
            break;
              
          default:
            System.out.println("WARNING: That motor doesn't exist!");
            break;
      }
    }
  
  private void setMotorPercent(CoralPlacerConstants.MOTOR motor, double percent){
    MotionMagicVelocityVoltage percentRequest = new MotionMagicVelocityVoltage(percent);
    switch(motor){

        case SHOOTER_MOTOR_1:
          shooterRollerTop.setControl(percentRequest);
          break;

        case SHOOTER_MOTOR_2:
          shooterRollerBottom.setControl(percentRequest);
          break;

        case WRIST_MOTOR:
          shooterWrist.setControl(percentRequest);
          break;

        case FEEDER_MOTOR:
          feederRoller.setControl(percentRequest);
          break;
            
        default:
          System.out.println("WARNING: That motor doesn't exist!");
          break;
    }
  }

  public void stopVoltage()
  {
    feederRoller.setVoltage(0);
    shooterRollerTop.setVoltage(0);
    shooterRollerBottom.setVoltage(0);
    //shooterWrist.setVoltage(0);
  }

  /**based on the control mode and value input into the parameters will call a method through a command
   * @param controlMode
   * @param value
   */

  // public void CoralPlacerRequest(Global.MODE controlMode, double value)
  // {
  //   switch(controlMode)
  //   {
  //     case VOLTAGE:
  //       setPlacerVoltage(value);
  //       break;
  //     case PERCENT:
  //       setPlacerPercentage(value);
  //       break;
  //     case POSITION:
  //       System.out.print("MODE: POSITION IS NOT AVAILABLE FOR CORAL SHOOTER");
  //       break;
  //     case BRAKE:
  //       System.out.print("MODE: BRAKE IS NOT AVAILABLE FOR CORAL SHOOTER");
  //       break;
  //   }
  // }

  // /** Sets voltage for placer motor
  //  * @param voltage amount of volts
  //  */
  // private void setPlacerVoltage(double voltage)
  // {
  //   VoltageOut voltageRequest = new VoltageOut(voltage);
  //   m_CoralPlacerMotor.setControl(voltageRequest);
  // }

  // /** Sets placer motor to work at a percentage of the max power it can.
  //  * @param percentage percentage of max power motor works at
  //  */
  // private void setPlacerPercentage(double percentage)
  // {
  //   DutyCycleOut dutyCycleRequest = new DutyCycleOut(percentage);
  //   m_CoralPlacerMotor.setControl(dutyCycleRequest);
  // }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
