// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Cobe-base configuration.
 */
public class Global
{
    /**
     * Different modes motors can call in commands
     * **** SOON TO BE REMOVED
     */
    public static enum MODE
    {
        PERCENT,
        VOLTAGE,
        POSITION,
        BRAKE
    }

    public static enum BUILD_TYPE
    {
        COMPETITION,
        DRIVETRAIN_DEBUG,
        _DEBUG
    }

    public static BUILD_TYPE ACTIVE_BUILD = BUILD_TYPE._DEBUG;
}