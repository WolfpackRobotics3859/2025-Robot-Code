// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

// Initializing constants for Climb.
public class ClimbConstants 
{
    // Insert wrist feed forward and default position values when determined.
    public static final double CLIMB_DEFAULT_WRIST_POSITION = 0;
    public static final double CLIMB_WRIST_FEED_FORWARD = 0;
    
    // Insert wrist gain values when determined.
    public static final Slot0Configs CLIMB_WRIST_MAIN_GAINS = new Slot0Configs()
    .withKP(0).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    // Insert wrist gain values when determined.
    public static final Slot0Configs CLIMB_WRIST_FOLLOWER_GAINS = new Slot0Configs()
    .withKP(0).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    // Insert Motion Magic configuration values when determined.
    public static final MotionMagicConfigs CLIMB_WRIST_MOTOR_MAIN_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0).withMotionMagicAcceleration(0);

    // Insert Motion Magic configuration values when determined.
    public static final MotionMagicConfigs CLIMB_WRIST_MOTOR_FOLLOWER_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0).withMotionMagicAcceleration(0);

    public static final TalonFXConfiguration CLIMB_WRIST_MAIN_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(CLIMB_WRIST_MOTOR_MAIN_MOTION_MAGIC_CONFIGURATION).withSlot0(CLIMB_WRIST_MAIN_GAINS);

    public static final TalonFXConfiguration CLIMB_WRIST_FOLLOWER_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(CLIMB_WRIST_MOTOR_FOLLOWER_MOTION_MAGIC_CONFIGURATION).withSlot0(CLIMB_WRIST_FOLLOWER_GAINS);
}
