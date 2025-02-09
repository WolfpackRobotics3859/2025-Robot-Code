package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ClawConstants {

        // Insert wrist gain values when determined.
    public static final Slot0Configs ALGAE_INTAKE_WRIST_GAINS = new Slot0Configs()
    .withKP(20).withKI(0).withKD(0).withGravityType(GravityTypeValue.Arm_Cosine);

    public static final int CLAW_ARM_PLACE_POSTION = 0;
    public static final int CLAW_ARM_STOW_POSITION = 0;
    public static final int CLAW_ROLLER_EXPEL_CORAL = 0;
    public static final int CLAW_ROLLER_INTAKE_CORAL = 0;
    public static final int CLAW_ROLLER_INTAKE_ALGAE = 0;
}
