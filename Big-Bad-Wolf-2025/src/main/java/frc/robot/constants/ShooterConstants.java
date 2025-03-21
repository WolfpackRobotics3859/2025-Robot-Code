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
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class ShooterConstants 
{
    // CORAL TOF SENSOR CONSTANTS
    public static final RangingMode CORAL_TOF_RANGING_MODE = RangingMode.Short;
    public static final double CORAL_TOF_IN_RANGE_THRESHOLD = 70;

    // CORAL MOTOR VOLTAGE
    public static final double CORAL_INTAKE_VOLTAGE = 2.75; // 2
    public static final double CORAL_DEPLOYMENT_VOLTAGE = 8; // 6
    public static final double CORAL_DEPLOYMENT_VOLTAGE_HIGH = 7; // 6
    
    // ALGAE MOTOR VOLTAGES   
    // Speed to use when cleaning the coral reef.
    public static final double ALGAE_SWEEPING_VOLTAGE = 4;
    // Speed to use when intaking algae from the ground.
    public static final double ALGAE_GROUND_INTAKING_VOLTAGE = -2.0;
    public static final double ALGAE_PROCESSOR_DEPLOYMENT_VOLTAGE = -6.0;
    // This will most likely be a variable algorithmic speed if we ever reach that level of automation.
    public static final double ALGAE_TROUGH_SHOOTING_VOLTAGE = 8;
    public static final double ALGAE_HOLDING_VOLTAGE = 1;

    public static final double WRIST_STOW_POSITION = -1.46; // -0.58
    public static final double WRIST_CORAL_DEPLOYMENT_POSITION_LOW = -2.5; // -3.5
    public static final double WRIST_CORAL_DEPLOYMENT_POSITION = -0.5; // -1.23 // -1.0
    public static final double WRIST_CORAL_INTAKE_POSITION = -0.4; // -0.62
    public static final double WRIST_ALGAE_INTAKE_POSITION = -11.0;
    public static final double WRIST_ALGAE_PROCESSOR_DEPLOYMENT_POSITION = -11; // -0.38
    public static final double WRIST_ALGAE_SHOOTING_POSITION = -0.15;
    public static final double WRIST_ALGAE_SWEEPING_POSITION = -0.5;

    // Maximum allowable magnitude deviation from setpoint when determining the end of the movement command.
    public static final double WRIST_POSITION_ERROR_TOLERANCE = 0.5;
    public static final double WRIST_POSITION_DERIVATIVE_TOLERANCE = 0.05;
    
    // WRIST MOTOR CONFIGS
    public static final MotorOutputConfigs WRIST_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final ClosedLoopGeneralConfigs WRIST_MOTOR_CLOSED_LOOP_GENERAL_CONFIG = new ClosedLoopGeneralConfigs()
                                                                                             .withContinuousWrap(false);

    public static final CurrentLimitsConfigs WRIST_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(80)
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
                                                                    .withKP(15)
                                                                    .withKS(0)
                                                                    .withKV(0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(80)
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

    // SHOOTER ALGAE MOTOR CONFIGS
    public static final MotorOutputConfigs SHOOTER_ALGAE_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final CurrentLimitsConfigs SHOOTER_ALGAE_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(120)
                                                                                    .withStatorCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final TalonFXConfiguration SHOOTER_ALGAE_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(SHOOTER_ALGAE_MOTOR_OUTPUT_CONFIG)
                                                                     .withCurrentLimits(SHOOTER_ALGAE_MOTOR_CURRENT_LIMIT_CONFIG);

    // SHOOTER CORAL MOTOR CONFIGS
    public static final MotorOutputConfigs SHOOTER_CORAL_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final CurrentLimitsConfigs SHOOTER_CORAL_MOTOR_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(120)
                                                                                    .withStatorCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final TalonFXConfiguration SHOOTER_CORAL_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(SHOOTER_CORAL_MOTOR_OUTPUT_CONFIG)
                                                                     .withCurrentLimits(SHOOTER_CORAL_MOTOR_CURRENT_LIMIT_CONFIG);
}
