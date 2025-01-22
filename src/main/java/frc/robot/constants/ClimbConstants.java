// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

// Initializing constants for climb.
public class ClimbConstants 
{
    // Insert wrist feed forward and default position values when determined.
    public static final double CLIMB_DEFAULT_WRIST_POSITION = 0;
    public static final double CLIMB_WRIST_FEED_FORWARD = 0;

    // Insert climb wrist gain values when determined.
    public static final Slot0Configs CLIMB_WRIST_GAINS = new Slot0Configs()
        .withKP(20).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    // Insert Motion Magic config values when determined.
    public static final MotionMagicConfigs CLIMB_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(30).withMotionMagicAcceleration(25);

    public static final TalonFXConfiguration CLIMB_WRIST_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(CLIMB_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION).withSlot0(CLIMB_WRIST_GAINS);
}