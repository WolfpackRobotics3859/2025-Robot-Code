// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralPlacerConstants;
import frc.robot.constants.Global;
import frc.robot.subsystems.CoralPlacer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  CoralPlacer m_CoralPlacer;
  
  /** Creates a new IntakeCoral. */
  public IntakeCoral(CoralPlacer p_CoralPlacer) {
    this.m_CoralPlacer = p_CoralPlacer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_CoralPlacer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.WRIST_MOTOR, Global.MODE.POSITION, CoralPlacerConstants.SHOOTER_INTAKE_WRIST_ANGLE);

    System.out.println("Back Laser: " + m_CoralPlacer.m_BackLaser.getRange());
    System.out.println("Front Ultrasonic: " + m_CoralPlacer.m_FrontUltrasonic.getRangeInches());

    // m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_1, Global.MODE.VOLTAGE, CoralPlacerConstants.SHOOTER_ROLLER_1_INTAKE_VELOCITY);
    // m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_2, Global.MODE.VOLTAGE, CoralPlacerConstants.SHOOTER_ROLLER_2_INTAKE_VELOCITY);
    if (!m_CoralPlacer.coralDetected())
    {
      System.out.println("Coral Detected");
      if(m_CoralPlacer.CoralInPlace())
      {
        System.out.println("Coral Intaking");
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_1, Global.MODE.VOLTAGE, CoralPlacerConstants.SHOOTER_ROLLER_1_INTAKE_VELOCITY);
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.FEEDER_MOTOR, Global.MODE.VOLTAGE, CoralPlacerConstants.SHOOTER_FEEDER_INTAKE_VOLTAGE);
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_2, Global.MODE.VOLTAGE, CoralPlacerConstants.SHOOTER_ROLLER_2_INTAKE_VELOCITY);
      }else{
        //coralplacerconstants are placeholder values because its a staticbrake which doesnt need any
        System.out.println("Coral In");
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_1, Global.MODE.BRAKE, CoralPlacerConstants.SHOOTER_ROLLER_1_INTAKE_VELOCITY);
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.FEEDER_MOTOR, Global.MODE.BRAKE, CoralPlacerConstants.SHOOTER_FEEDER_INTAKE_VOLTAGE);
        m_CoralPlacer.setMotor(CoralPlacerConstants.MOTOR.SHOOTER_MOTOR_2, Global.MODE.BRAKE, CoralPlacerConstants.SHOOTER_ROLLER_2_INTAKE_VELOCITY);
      }
    }else{
      System.out.println("Voltage Stopped (IntakeCoral)");
      m_CoralPlacer.stopVoltage();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
