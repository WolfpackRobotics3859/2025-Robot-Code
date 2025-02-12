package frc.robot.subsystems.elevator;

public class ElevatorRequest 
{
    private ElevatorRequestType requestType;
    private double requestValue;

    public ElevatorRequest()
    {
        requestType = ElevatorRequestType.NOOP;
        requestValue = 0;
    }

    public ElevatorRequest VoltageOut(double voltage)
    {
        this.requestType = ElevatorRequestType.VOLTAGE;
        this.requestValue = voltage;
        return this;
    }

    public ElevatorRequest PercentOutput(double percent)
    {
        this.requestType = ElevatorRequestType.PERCENT;
        this.requestValue = percent;
        return this;
    }

    public ElevatorRequest Position(double position)
    {
        this.requestType = ElevatorRequestType.POSITION;
        this.requestValue = position;
        return this;
    }

    public ElevatorRequest Brake()
    {
        this.requestType = ElevatorRequestType.BRAKE;
        return this;
    }

    public ElevatorRequestType GetType()
    {
        return this.requestType;
    }

    public double GetValue()
    {
        return this.requestValue;
    }
}
