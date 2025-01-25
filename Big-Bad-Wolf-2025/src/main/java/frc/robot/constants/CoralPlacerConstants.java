// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class CoralPlacerConstants 
{
    //All values are filler values and until further testing
    public static final double CORAL_PLACER_INTAKE_VOLTAGE = 4;
    public static final double CORAL_PLACER_PURGE_VOLTAGE = 8;
    public static final double CORAL_PLACER_UNJAM_VOLTAGE = -1;
    public static final double CORAL_PLACER_WRIST_PLACE_POSITION = 10;
    public static final double CORAL_PLACER_WRIST_INTAKE_POSITION = 20;
    public static final double CORAL_PLACER_WRIST_STOW_POSITION = 30;
    public static final double FRONT_LASER_LIMIT = 250;
    public static final double BACK_LASER_LIMIT = 250;
    public static final int INTAKE_TIME_THRESHOLD = 5;
    public static final int UNJAM_TIME_THRESHOLD = 8;

    public static enum MOTOR
    {
        PLACER_ROLLER,
        PLACER_WRIST
    }
}
