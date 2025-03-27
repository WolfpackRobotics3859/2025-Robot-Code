package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class WristConstants 
{
    public static final String NAME = "WRIST";

    public enum ANGLES 
    {
        ZERO(-0.077637),
        TRAVEL(-0.065918),
        DEPLOY_LOW(-0.053955),
        DEPLOY_HIGH(-0.07),
        INTAKE(-0.077637),
        SWEEP(0.042561),
        BARGE(-0.065918),
        PROCESSOR(0.107666),
        GRAB(0.171865);

        private double levelValue;

        ANGLES(double value)
        {
            this.levelValue = value;
        }

        public double getValue()
        {
            return levelValue;
        }
    }

    // Maximum allowable magnitude deviation from setpoint when determining the end of the movement command.
    public static final double WRIST_POSITION_ERROR_TOLERANCE = 0.5;
    public static final double WRIST_POSITION_DERIVATIVE_TOLERANCE = 0.05;

    public static final MagnetSensorConfigs MAG_SENSOR_CONFIGS = new MagnetSensorConfigs().withMagnetOffset(-0.471435546875);

    public static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration().withMagnetSensor(MAG_SENSOR_CONFIGS);
    
    // WRIST MOTOR CONFIGS
    public static final MotorOutputConfigs WRIST_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.CounterClockwise_Positive)
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
                                                                    .withKP(35)
                                                                    .withKS(0)
                                                                    .withKV(0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(80)
                                                                                .withMotionMagicCruiseVelocity(300)
                                                                                .withMotionMagicExpo_kA(0.1)
                                                                                .withMotionMagicExpo_kV(0.12)
                                                                                .withMotionMagicJerk(0);
    
    public static final FeedbackConfigs MOTOR_FEEDBACK_CONFIGS = new FeedbackConfigs()
                                                                     .withFeedbackRemoteSensorID(Hardware.WRIST_ENCODER)
                                                                     .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
    
    public static final TalonFXConfiguration WRIST_MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(WRIST_MOTOR_OUTPUT_CONFIG)
                                                                     .withClosedLoopGeneral(WRIST_MOTOR_CLOSED_LOOP_GENERAL_CONFIG)
                                                                     .withCurrentLimits(WRIST_MOTOR_CURRENT_LIMIT_CONFIG)
                                                                     .withSlot0(WRIST_MOTOR_SLOT_0_CONFIG)
                                                                     .withMotionMagic(WRIST_MOTOR_MOTION_MAGIC_CONFIG)
                                                                     .withFeedback(MOTOR_FEEDBACK_CONFIGS);
}
