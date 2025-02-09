package frc.robot.subsystems.claw;

public class ClawRequest {
    private ClawRequestType clawRequestType;
    private double requestValue;

    public ClawRequest() {
        clawRequestType = ClawRequestType.NOOP;
        requestValue = 0;
    }

    public ClawRequest position(double position) {
        this.clawRequestType = ClawRequestType.POSITION;
        this.requestValue = position;
        return this;
    }

    public ClawRequest voltageOut(double voltage) {
        this.clawRequestType = ClawRequestType.VOLTAGE;
        this.requestValue = voltage;
        return this;
    }

    public ClawRequest percentOut(double percent) {
        this.clawRequestType = ClawRequestType.PERCENT;
        this.requestValue = percent;
        return this;
    }

    public ClawRequest brake() {
        this.clawRequestType = ClawRequestType.BRAKE;
        return this;
    }

    public ClawRequestType getClawRequestType() {
        return clawRequestType;
    }

    public double getValue() {
        return requestValue;
    }
}
