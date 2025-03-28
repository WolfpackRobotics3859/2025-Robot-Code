package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class WheelConstants
{
    public static final String NAME = "WHEELS";

    public static final int TOF_FORWARD_ID = Hardware.CORAL_FORWARD_TOF_SENSOR;
    public static final int TOF_REAR_ID = Hardware.CORAL_REAR_TOF_SENSOR;

    public static final RangingMode TOF_RANGING_MODE = RangingMode.Short;
    public static final double TOF_IN_RANGE_THRESHOLD = 70;

    public static final double SAFETY_SIGNAL_DEBOUNCE_SECONDS = 0.0625;
    public static final double MOTOR_RESISTANCE_SENSE_DEBOUNCE_SECONDS = 0.125;

    public static final double ALGAE_RESISTANCE_VELOCITY_THRESHOLD = 0.1;

    public enum VoltageSpeeds
    {
        ZERO(0),
        INTAKE(1.5),
        SWEEP(3),
        FORWARD_CENTERING(1.0),
        REVERSE_CENTERING(-1.5),
        DEPLOY_LOW(6), // 6
        DEPLOY_HIGH(8),
        BARGE(-12);

        private double value;

        VoltageSpeeds(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }
    }

    public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                                                                          .withInverted(InvertedValue.Clockwise_Positive)
                                                                          .withNeutralMode(NeutralModeValue.Brake)
                                                                          .withPeakForwardDutyCycle(1)
                                                                          .withPeakReverseDutyCycle(-1);

    public static final Slot0Configs SLOT_0_CONFIG = new Slot0Configs()
                                                                    .withKA(0)
                                                                    .withKD(0)
                                                                    .withKG(0)
                                                                    .withKI(0)
                                                                    .withKP(5)
                                                                    .withKS(0)
                                                                    .withKV(0)
                                                                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(80)
                                                                                .withMotionMagicCruiseVelocity(300)
                                                                                .withMotionMagicExpo_kA(0)
                                                                                .withMotionMagicExpo_kV(0)
                                                                                .withMotionMagicJerk(0);
    

    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                                                                                    .withStatorCurrentLimit(120)
                                                                                    .withStatorCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLimit(40)
                                                                                    .withSupplyCurrentLimitEnable(false)
                                                                                    .withSupplyCurrentLowerLimit(30)
                                                                                    .withSupplyCurrentLowerTime(1);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
                                                                     .withMotorOutput(OUTPUT_CONFIG)
                                                                     .withCurrentLimits(CURRENT_LIMIT_CONFIG)
                                                                     .withSlot0(SLOT_0_CONFIG)
                                                                     .withMotionMagic(MOTION_MAGIC_CONFIG);
}
