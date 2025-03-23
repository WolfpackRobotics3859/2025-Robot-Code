// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

// Initializing constants for Climb.
public class ClimbConstants 
{
    public static final double CLIMB_WRIST_VOLTAGE = 2;
    public static final double CLIMB_ROLLER_VOLTAGE = 2;

    public static final MotorOutputConfigs CLIMB_WRIST_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    public static final MotorOutputConfigs CLIMB_ROLLER_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    public static final MotorOutputConfigs CORAL_FUNNEL_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    public static final TalonFXConfiguration WRIST_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CLIMB_WRIST_MOTOR_OUTPUT);

    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CLIMB_ROLLER_MOTOR_OUTPUT);
    
    public static final TalonFXConfiguration FEET_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CORAL_FUNNEL_MOTOR_OUTPUT);
}
