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

/** 
 * Where all constants related to elevator are declared.
 */
public class ElevatorConstants 
{
    public static final double ELEVATOR_ZERO_POSITION = 0;
    public static final double ELEVATOR_LEVEL_ONE = 0;
    public static final double ELEVATOR_LEVEL_TWO = 26;
    public static final double ELEVATOR_LEVEL_THREE = 38;
    public static final double ELEVATOR_LEVEL_FOUR = 60;
    public static final double CORAL_INTAKING_LEVEL = 28;

    public static final double ELEVATOR_UP_VOLTAGE = 3;
    public static final double ELEVATOR_DOWN_VOLTAGE = -1.5;

    public static final double HOMING_VOLTAGE = -0.5;

    public enum LEVELS 
    {
        HOME(ELEVATOR_ZERO_POSITION),
        ONE(ELEVATOR_LEVEL_ONE),
        TWO(ELEVATOR_LEVEL_TWO),
        THREE(ELEVATOR_LEVEL_THREE),
        FOUR(ELEVATOR_LEVEL_FOUR),
        CORAL_INTAKE(CORAL_INTAKING_LEVEL);

        private double levelValue;

        LEVELS(double value)
        {
            this.levelValue = value;
        }

        public double getValue()
        {
            return levelValue;
        }
    }

    // TalonFX Configs

    // LEFT MOTOR CONFIGS
    public static final MotorOutputConfigs LEFT_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final ClosedLoopGeneralConfigs LEFT_MOTOR_CLOSED_LOOP_GENERAL_CONFIG = new ClosedLoopGeneralConfigs()
                                                                                             .withContinuousWrap(false);

    public static final CurrentLimitsConfigs LEFT_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(100)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final Slot0Configs LEFT_MOTOR_SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withGravityType(GravityTypeValue.Elevator_Static)
                                                                    .withKA(0)
                                                                    .withKD(0)
                                                                    .withKG(0)
                                                                    .withKI(0)
                                                                    .withKP(15)
                                                                    .withKS(0)
                                                                    .withKV(0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs LEFT_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(20)
                                                                                .withMotionMagicCruiseVelocity(40)
                                                                                .withMotionMagicExpo_kA(0.1)
                                                                                .withMotionMagicExpo_kV(0.12)
                                                                                .withMotionMagicJerk(0);
    
    public static final HardwareLimitSwitchConfigs LEFT_MOTOR_HARDWARE_LIMIT_SWITCH_CONFIG = new HardwareLimitSwitchConfigs()
                                                                                                 .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS1)
                                                                                                 .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                                                                                                 .withReverseLimitAutosetPositionValue(0)
                                                                                                 .withReverseLimitAutosetPositionEnable(true)
                                                                                                 .withReverseLimitEnable(true)
                                                                                                 .withReverseLimitRemoteSensorID(Hardware.CANDI_0);

    public static final TalonFXConfiguration LEFT_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIG)
                                                                     .withClosedLoopGeneral(LEFT_MOTOR_CLOSED_LOOP_GENERAL_CONFIG)
                                                                     .withCurrentLimits(LEFT_MOTOR_CURRENT_LIMIT_CONFIG)
                                                                     .withSlot0(LEFT_MOTOR_SLOT_0_CONFIG)
                                                                     .withMotionMagic(LEFT_MOTOR_MOTION_MAGIC_CONFIG)
                                                                     .withHardwareLimitSwitch(LEFT_MOTOR_HARDWARE_LIMIT_SWITCH_CONFIG);

    // RIGHT MOTOR CONFIGS
    public static final MotorOutputConfigs RIGHT_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final CurrentLimitsConfigs RIGHT_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(100)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(40)
                                                                                    .withSupplyCurrentLowerTime(1);
    
    public static final HardwareLimitSwitchConfigs RIGHT_MOTOR_HARDWARE_LIMIT_SWITCH_CONFIG = new HardwareLimitSwitchConfigs()
                                                                                                 .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS1)
                                                                                                 .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                                                                                                 .withReverseLimitEnable(true)
                                                                                                 .withReverseLimitRemoteSensorID(Hardware.CANDI_0);

    public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(RIGHT_MOTOR_OUTPUT_CONFIG)
                                                                     .withCurrentLimits(RIGHT_MOTOR_CURRENT_LIMIT_CONFIG)
                                                                     .withHardwareLimitSwitch(RIGHT_MOTOR_HARDWARE_LIMIT_SWITCH_CONFIG);
}
