// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

// Initializing constants for Algae Intake.
public class IntakeConstants
{
    // Insert wrist feed forward and default position values when determined.
    public static final double INTAKE_WRIST_FEED_FORWARD = 0;

    public static final double INTAKE_WRIST_DEFAULT_POSITION = 0;
    public static final double INTAKE_WRIST_GROUND_POSITION = 7.48;
    public static final double INTAKE_WRIST_PROCESSING_POSITION = 0;

    public static final double INTAKE_ROLLER_DEFAULT_VOLTAGE = 0;
    public static final double INTAKE_ROLLER_VOLTAGE = 0;
    public static final double INTAKE_ROLLER_PROCESSING_VOLTAGE = 0;

    // Insert roller gain values when determined.
    public static final Slot0Configs INTAKE_ROLLER_GAINS = new Slot0Configs()
    .withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    // Insert wrist gain values when determined.
    public static final Slot0Configs INTAKE_WRIST_GAINS = new Slot0Configs()
    .withKP(20).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    // Insert Motion Magic configuration values when determined.
    public static final MotionMagicConfigs INTAKE_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(300).withMotionMagicAcceleration(100);

    public static final TalonFXConfiguration INTAKE_WRIST_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(INTAKE_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION).withSlot0(INTAKE_WRIST_GAINS);
}