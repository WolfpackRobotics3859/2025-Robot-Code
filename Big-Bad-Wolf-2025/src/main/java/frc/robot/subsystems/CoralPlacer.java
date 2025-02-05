// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;
import frc.robot.subsystems.placer.PlacerRequest;

public class CoralPlacer extends SubsystemBase
{
  public final TalonFX m_CoralPlacerRollerMotor;
  public final TalonFX m_CoralPlacerWristMotor;

  // public final TimeOfFlight m_FrontLaser;
  // public final TimeOfFlight m_BackLaser;

  // public final Timer intakeTimer;

  private PlacerRequest m_CurrenRequest;
  private SysIdRoutine m_SysIdRoutine;

  /**
   * Creates new CoralPlacer subsystem; initializes motors and sensors
   */
  public CoralPlacer()
  {
    m_CoralPlacerRollerMotor = new TalonFX(Hardware.CORAL_PLACER_ROLLER_MOTOR_ID);
    m_CoralPlacerWristMotor = new TalonFX(Hardware.CORAL_PLACER_WRIST_MOTOR_ID);

    // m_FrontLaser = new TimeOfFlight(Hardware.CORAL_PLACER_FRONT_LASER_ID);
    // m_BackLaser = new TimeOfFlight(Hardware.CORAL_PLACER_BACK_LASER_ID);
    
    //intakeTimer = new Timer();
  }

  // /** 
  //  * Boolean for detecting whether an object is within front laser's detected range
  //  * 
  //  * @return True if the laser detected range is less than its normal value (if it detects something)
  //  */
  // public boolean frontLaserDetect()
  // {
  //   return this.m_FrontLaser.getRange() < CoralPlacerConstants.FRONT_LASER_LIMIT;
  // }

  // /** 
  //  * Boolean for detecting whether an object is within back laser's detected range
  //  * 
  //  * @return True if the laser's detected range is less than its normal value (if it detects something)
  //  */
  // public boolean backLaserDetect()
  // {
  //   return this.m_BackLaser.getRange() < CoralPlacerConstants.BACK_LASER_LIMIT;
  // }

  // /** 
  //  * Logic for accepting coral from feeder using outputs form both TimeOfFLight sensors.
  //  */
  // public void IntakeCoral()
  // {
  //   if (frontLaserDetect())
  //   {
  //     // If front laser detects: reset and stop timer
  //     intakeTimer.reset();
  //     if (backLaserDetect())
  //     {
  //       // If front laser and back laser detect then there's two corals in intake: purge coral
  //       applyPlacerVoltage(CoralPlacerConstants.CORAL_PLACER_PURGE_VOLTAGE, m_CoralPlacerRollerMotor);
  //     }
  //     else
  //     {
  //       // If front laser detects but back doesn't: brake to hold coral
  //       brakePlacer(m_CoralPlacerRollerMotor);
  //     }
  //   }
  //   else if (backLaserDetect())
  //   {
  //     // If back laser detects: start intakeTimer
  //     intakeTimer.start();

  //     if (intakeTimer.get() >= CoralPlacerConstants.INTAKE_TIME_THRESHOLD && intakeTimer.get() < CoralPlacerConstants.UNJAM_TIME_THRESHOLD)
  //     {
  //       // If back laser detect and timer is greater than or equal to INTAKE_TIME_THRESHOLD and less than UNJAM_TIME_THRESHOLD: unjam
  //       applyPlacerVoltage(CoralPlacerConstants.CORAL_PLACER_UNJAM_VOLTAGE, m_CoralPlacerRollerMotor);

  //     }
  //     else if (intakeTimer.get() >= CoralPlacerConstants.UNJAM_TIME_THRESHOLD)
  //     {
  //       /* If back laser detect and timer greater or equal to UNJAM_TIME_THRESHOLD: intake
  //       We only reset timer because loop will trigger else if (backLaserDetect) and instantly go to else statement*/
  //       intakeTimer.reset();
  //     }
  //     else
  //     {
  //       // If back laser detect and timer less than INTAKE_TIME_THRESHOLD: intake
  //       applyPlacerVoltage(CoralPlacerConstants.CORAL_PLACER_INTAKE_VOLTAGE, m_CoralPlacerRollerMotor)
  //     }
  //   }
  //   else
  //   {
  //     if (intakeTimer.get() >= CoralPlacerConstants.INTAKE_TIME_THRESHOLD)
  //     {
  //       // If neither lasers detect and timer is over threshold for intaking: stop intake, stop and reset timer
  //       applyPlacerVoltage(0, m_CoralPlacerRollerMotor);
  //       intakeTimer.reset();
  //     }
  //     else
  //     {
  //       // If neither lasers detect but timer isn't over threshold for intaking: intake
  //       applyPlacerVoltage(CoralPlacerConstants.CORAL_PLACER_INTAKE_VOLTAGE, m_CoralPlacerRollerMotor);
  //     }
  //   }
  // }

  public Command applyWristVoltage(double voltage)
  {
    return this.startEnd(() -> m_CoralPlacerWristMotor.setControl(new VoltageOut(voltage)), () -> m_CoralPlacerWristMotor.setControl(new StaticBrake()));
  }

  public Command applyShooterVoltage(double voltage)
  {
    return this.startEnd(() -> m_CoralPlacerRollerMotor.setControl(new VoltageOut(voltage)), () -> m_CoralPlacerRollerMotor.setControl(new VoltageOut(0)));
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


  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic()
  {
    // Empty as of now
  }
}
