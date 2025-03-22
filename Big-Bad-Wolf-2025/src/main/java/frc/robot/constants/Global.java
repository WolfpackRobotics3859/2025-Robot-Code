// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Cobe-base configuration.
 */
public class Global
{
    public static enum BUILD_TYPE
    {
        COMPETITION,
        COMPETITION_NO_VISION,
        DRIVETRAIN_DEBUG,
        ELEVATOR_DEBUG,
        SHOOTER_DEBUG,
        INTAKE_DEBUG,
        CLIMB_DEBUG,
        ELEVATOR_SHOOTER_DEBUG,
        CO_ELEVATOR_SHOOTER_DEBUG
    }

    public static BUILD_TYPE ACTIVE_BUILD = BUILD_TYPE.COMPETITION;
}