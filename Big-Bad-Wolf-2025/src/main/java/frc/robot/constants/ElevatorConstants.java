// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.utilities.DataStuff;

/** 
 * Where all constants related to elevator are declared.
 */
public class ElevatorConstants 
{
    public static final double ELEVATOR_ZERO_POSITION = 0;
    public static final double ELEVATOR_HOME_POSITION = 17.29;
    public static final double ELEVATOR_LEVEL_ONE = 8.643;
    public static final double ELEVATOR_LEVEL_TWO = 26.943;  // 50 // 46
    public static final double ELEVATOR_LEVEL_THREE = 43.79; // 78  
    public static final double ELEVATOR_LEVEL_FOUR = 60.5; 
    public static final double CORAL_INTAKING_LEVEL = 1.1524;
    public static final double ALGAE_PROCESSING_POSITION = 0.576;
    public static final double ALGAE_BARGE_POSITION = 59.0;
    public static final double LOW_CLEAN = 15;
    public static final double HIGH_CLEAN = 37.05;

    // Maximum allowable magnitude deviation from setpoint when determining the end of the movement command.
    public static final double POSITION_ERROR_TOLERANCE = 0.57;
    public static final double POSITION_DERIVATIVE_TOLERANCE = 0.05;

    // Max Height should be around 100-105

    public static final double ELEVATOR_UP_VOLTAGE = 3;
    public static final double ELEVATOR_DOWN_VOLTAGE = -1.5;
 
    public static final double HOMING_VOLTAGE = -0.5;

    public enum LEVELS 
    {
        ZERO(ELEVATOR_ZERO_POSITION),
        HOME(ELEVATOR_HOME_POSITION),
        ONE(ELEVATOR_LEVEL_ONE),
        TWO(ELEVATOR_LEVEL_TWO),
        THREE(ELEVATOR_LEVEL_THREE),
        FOUR(ELEVATOR_LEVEL_FOUR),
        CORAL_INTAKE(CORAL_INTAKING_LEVEL),
        LOW_ALGAE(LOW_CLEAN),
        HIGH_ALGAE(HIGH_CLEAN),
        ALGAE_PROCESS(ALGAE_PROCESSING_POSITION),
        ALGAE_BARGE(ALGAE_BARGE_POSITION);
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
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final ClosedLoopGeneralConfigs LEFT_MOTOR_CLOSED_LOOP_GENERAL_CONFIG = new ClosedLoopGeneralConfigs()
                                                                                             .withContinuousWrap(false);

    public static final CurrentLimitsConfigs LEFT_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(120)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final Slot0Configs LEFT_MOTOR_SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withGravityType(GravityTypeValue.Elevator_Static)
                                                                    .withKA(0.014085)
                                                                    .withKD(0)
                                                                    .withKG(0.2532)
                                                                    .withKI(0)
                                                                    .withKP(10)
                                                                    .withKS(0.10171)
                                                                    .withKV(0.11636)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs LEFT_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(125) // 175
                                                                                .withMotionMagicCruiseVelocity(700) // 800
                                                                                .withMotionMagicExpo_kA(0.014085)
                                                                                .withMotionMagicExpo_kV(0.11636)
                                                                                .withMotionMagicJerk(0);

    public static final TalonFXConfiguration ELEVATOR_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIG)
                                                                     .withClosedLoopGeneral(LEFT_MOTOR_CLOSED_LOOP_GENERAL_CONFIG)
                                                                     .withCurrentLimits(LEFT_MOTOR_CURRENT_LIMIT_CONFIG)
                                                                     .withSlot0(LEFT_MOTOR_SLOT_0_CONFIG)
                                                                     .withMotionMagic(LEFT_MOTOR_MOTION_MAGIC_CONFIG);
}
