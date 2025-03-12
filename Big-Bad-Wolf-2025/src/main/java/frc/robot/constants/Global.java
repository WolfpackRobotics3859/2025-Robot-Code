package frc.robot.constants;

/**
 * Cobe-base configuration.
 */
public class Global
{
    public static enum BUILD_TYPE
    {
        COMPETITION,
        DRIVETRAIN_DEBUG,
        ELEVATOR_DEBUG,
        SHOOTER_DEBUG,
        INTAKE_DEBUG,
        CLIMB_DEBUG,
        ELEVATOR_SHOOTER_DEBUG
    }

    public static BUILD_TYPE ACTIVE_BUILD = BUILD_TYPE.ELEVATOR_DEBUG;
}