// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class CoralPlacerConstants
{
    //  Sensor limits
    public static final double BACK_LASER_LIMIT = 250;
    public static final double MIN_BACK_LASER_LIMIT = 0;

    public static final double SHOOTER_ROLLER_1_INTAKE_VELOCITY = -4;
    public static final double SHOOTER_ROLLER_2_INTAKE_VELOCITY = -4;
    public static final double SHOOTER_FEEDER_INTAKE_VOLTAGE = -1.5;
    public static final double SHOOTER_INTAKE_WRIST_ANGLE = 0.53;


    //  Temporary for testing
    public enum MOTOR
    {  
        SHOOTER_MOTOR_1,
        SHOOTER_MOTOR_2,
        WRIST_MOTOR,
        FEEDER_MOTOR
    }
}
