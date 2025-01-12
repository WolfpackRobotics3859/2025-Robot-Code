// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;


/** Add your docs here. */
public class ElevatorConstants
{
    public static final double BAR = 0.481201171875;
    public static final double BAR_TOP_CLEAR = 0.625; // The first position where the shooter can freely rotate again above the crossbar.
    public static final double BAR_BOTTOM_CLEAR = 0.3330078125; // The first position where the shooter can freely rotate again below the crossbar.
    public static final double CLOSED_LOOP_ERROR_TOLERANCE = 0.005;

    // Limit Values
    public static final double ELEVATOR_TOP_LIMIT = 0.82;
    public static final double ELEVATOR_BOTTOM_LIMIT = 0.06;

    // Misc Values
    public static final double ELEVATOR_FEED_FORWARD = 0.2;
}
