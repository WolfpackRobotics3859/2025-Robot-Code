// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ElevatorConstants 
{
    public static final String NAME = "ELEVATOR";
    // Maximum allowable magnitude deviation from setpoint when determining the end of the movement command.
    public static final double POSITION_ERROR_TOLERANCE = 1;
    public static final double POSITION_DERIVATIVE_TOLERANCE = 0.05;

    public enum HEIGHTS 
    {
        ZERO(0),
        TRAVEL(5),
        ONE(10),
        TWO(19.5), // 20
        THREE(34),
        FOUR(59), // 58
        INTAKE(1),
        LOW_CLEAN(15),
        HIGH_CLEAN(30),
        BARGE(58),
        PROCESS(1);

        private double levelValue;

        HEIGHTS(double value)
        {
            this.levelValue = value;
        }

        public double getValue()
        {
            return levelValue;
        }
    }

    // LEFT MOTOR CONFIGS
    public static final MotorOutputConfigs LEFT_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final ClosedLoopGeneralConfigs LEFT_MOTOR_CLOSED_LOOP_GENERAL_CONFIG = new ClosedLoopGeneralConfigs()
                                                                                             .withContinuousWrap(false);

    public static final CurrentLimitsConfigs LEFT_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(70)
                                                                                    .withStatorCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLimit(20)
                                                                                    .withSupplyCurrentLimitEnable(true)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final Slot0Configs LEFT_MOTOR_SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withGravityType(GravityTypeValue.Elevator_Static)
                                                                    .withKA(0.014085)
                                                                    .withKD(0)
                                                                    .withKG(0.2532)
                                                                    .withKI(0)
                                                                    .withKP(15)
                                                                    .withKS(0.10171)
                                                                    .withKV(0.11636)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs LEFT_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(300)
                                                                                .withMotionMagicCruiseVelocity(1200)
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
