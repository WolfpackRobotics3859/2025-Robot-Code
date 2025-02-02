// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.placer;

import frc.robot.subsystems.placer.PlacerRequestType.PLACER_REQUEST_TYPE;

public class PlacerRequest 
{
    private PLACER_REQUEST_TYPE requestType;
    private double requestValue;

    public PlacerRequest()
    {
        requestType = PLACER_REQUEST_TYPE.NOOP;
        requestValue = 0;
    }

    public PlacerRequest VoltageOut(double voltage)
    {
        this.requestType = PLACER_REQUEST_TYPE.VOLTAGE;
        this.requestValue = voltage;
        return this;
    }

    public PlacerRequest PercentOutput(double percent)
    {
        this.requestType = PLACER_REQUEST_TYPE.PERCENT;
        this.requestValue = percent;
        return this;
    }

    public PlacerRequest Position(double position)
    {
        this.requestType = PLACER_REQUEST_TYPE.POSITION;
        this.requestValue = position;
        return this;
    }

    public PlacerRequest Brake()
    {
        this.requestType = PLACER_REQUEST_TYPE.BRAKE;
        return this;
    }

    public PLACER_REQUEST_TYPE GetType()
    {
        return this.requestType;
    }

    public double GetValue()
    {
        return this.requestValue;
    }
}

