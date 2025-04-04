// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

// Initializing constants for Climb.
public class ClimbConstants 
{
    public static final double CLIMB_WRIST_VOLTAGE = 2;
    public static final double CLIMB_ROLLER_VOLTAGE = -8;

    public static final double FUNNEL_RELEASE_VOLTAGE = 2;
    public static final double FOOT_RELEASE_VOTLAGE = -2;

    public static final double CLIMB_RESTING_POSITION = 0.060547;
    public static final double CLIMB_CLIMB_POSITION = 0.085;
    public static final double CLIMB_TAKING_POSITION = -0.130752;

    public static final double CLIMB_STOP_POSITION = 0.09;

    public static final double TOF_IN_RANGE_THRESHOLD = 140.0;

    public static final MotorOutputConfigs CLIMB_WRIST_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);
    public static final MotorOutputConfigs CLIMB_ROLLER_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
    public static final MotorOutputConfigs CORAL_FUNNEL_MOTOR_OUTPUT = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);


    public static final MagnetSensorConfigs MAG_SENSOR_CONFIGS = new MagnetSensorConfigs().withMagnetOffset(0.217529296875);

    public static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration().withMagnetSensor(MAG_SENSOR_CONFIGS);

    public static final FeedbackConfigs WRIST_FEEDBACK_CONFIGS = new FeedbackConfigs()
                                                                     .withFeedbackRemoteSensorID(Hardware.CLIMB_ENCODER)
                                                                     .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

    public static final CurrentLimitsConfigs WRIST_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(120)
                                                                                    .withStatorCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLimit(20)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final Slot0Configs WRIST_MOTOR_SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withGravityType(GravityTypeValue.Elevator_Static)
                                                                    .withKA(0.0)
                                                                    .withKD(0.0)
                                                                    .withKG(0.0)
                                                                    .withKI(0)
                                                                    .withKP(400)
                                                                    .withKS(0.0)
                                                                    .withKV(0.0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(5)
                                                                                .withMotionMagicCruiseVelocity(5)
                                                                                .withMotionMagicExpo_kA(0.014085)
                                                                                .withMotionMagicExpo_kV(0.11636)
                                                                                .withMotionMagicJerk(0);

    public static final TalonFXConfiguration WRIST_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CLIMB_WRIST_MOTOR_OUTPUT)
        .withSlot0(WRIST_MOTOR_SLOT_0_CONFIG)
        .withMotionMagic(WRIST_MOTOR_MOTION_MAGIC_CONFIG)
        .withFeedback(WRIST_FEEDBACK_CONFIGS)
        .withCurrentLimits(WRIST_MOTOR_CURRENT_LIMIT_CONFIG);

    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CLIMB_ROLLER_MOTOR_OUTPUT);
    
    public static final TalonFXConfiguration FEET_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(CORAL_FUNNEL_MOTOR_OUTPUT);
}
