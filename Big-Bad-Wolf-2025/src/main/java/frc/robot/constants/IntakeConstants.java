// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// Initializing constants for Algae Intake.
public class IntakeConstants
{
    public static final double WRIST_STOW_POSITION = 0;
    public static final double WRIST_INTAKING_MID_EXTENSION = 3;
    public static final double WRIST_INTAKING_POSITION = 7.48;
    public static final double WRIST_PROCESSING_POSITION = 0;

    public static final double INTAKING_CLEARING_VOLTAGE = 1.5;
    public static final double INTAKING_VOLTAGE = -3;

    public static final double PROCESSING_VOLTAGE = -3.0;

    // INTAKE WRIST CONFIGS
    public static final MotorOutputConfigs WRIST_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final ClosedLoopGeneralConfigs WRIST_MOTOR_CLOSED_LOOP_GENERAL_CONFIG = new ClosedLoopGeneralConfigs()
                                                                                             .withContinuousWrap(false);

    public static final CurrentLimitsConfigs WRIST_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(100)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final Slot0Configs WRIST_MOTOR_SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withGravityType(GravityTypeValue.Arm_Cosine)
                                                                    .withKA(0)
                                                                    .withKD(0)
                                                                    .withKG(0)
                                                                    .withKI(0)
                                                                    .withKP(20)
                                                                    .withKS(0)
                                                                    .withKV(0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(100)
                                                                                .withMotionMagicCruiseVelocity(300)
                                                                                .withMotionMagicExpo_kA(0.1)
                                                                                .withMotionMagicExpo_kV(0.12)
                                                                                .withMotionMagicJerk(0);


    public static final TalonFXConfiguration WRIST_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(WRIST_MOTOR_OUTPUT_CONFIG)
                                                                     .withClosedLoopGeneral(WRIST_MOTOR_CLOSED_LOOP_GENERAL_CONFIG)
                                                                     .withCurrentLimits(WRIST_MOTOR_CURRENT_LIMIT_CONFIG)
                                                                     .withSlot0(WRIST_MOTOR_SLOT_0_CONFIG)
                                                                     .withMotionMagic(WRIST_MOTOR_MOTION_MAGIC_CONFIG);

    // INTAKE ROLLER CONFIGS
    public static final MotorOutputConfigs ROLLER_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final CurrentLimitsConfigs ROLLER_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(100)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(ROLLER_MOTOR_OUTPUT_CONFIG)
                                                                     .withCurrentLimits(ROLLER_MOTOR_CURRENT_LIMIT_CONFIG);
}