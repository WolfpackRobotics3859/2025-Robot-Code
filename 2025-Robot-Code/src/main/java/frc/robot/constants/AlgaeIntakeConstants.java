// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

// Initializing constants for Algae Intake.
public class AlgaeIntakeConstants
{
    // Insert wrist feed forward and default position values when determined.
    public static final double ALGAE_INTAKE_WRIST_FEED_FORWARD = 2.5;
    public static final double ALGAE_INTKAE_WRIST_DEFAULT_POSITION = 2.5;

    // Insert roller gain values when determined.
    public static final Slot0Configs ALGAE_INTAKE_ROLLER_GAINS = new Slot0Configs()
    .withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    // Insert wrist gain values when determined.
    public static final Slot0Configs ALGAE_INTAKE_WRIST_GAINS = new Slot0Configs()
    .withKP(20).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs ALGAE_INTAKE_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0).withMotionMagicAcceleration(0);

    public static final TalonFXConfiguration ALGAE_INTAKE_WRIST_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(ALGAE_INTAKE_WRIST_MOTOR_MOTION_MAGIC_CONFIGURATION).withSlot0(ALGAE_INTAKE_WRIST_GAINS);

}