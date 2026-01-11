package frc.robot.utilities.subsystemManager;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An object that stores subsystem info when a subsystem is added to the {@link SubsystemManager}.
 */
public class SubsystemAddedEvent 
{
    private Subsystem m_Subsystem;
    
    /**
     * Creates a new subsystem added event object.
     * @param subsystem the subsystem to attach to the event object.
     */
    public SubsystemAddedEvent(Subsystem subsystem)
    {
        this.m_Subsystem = subsystem;
    }

    /**
     * Gets the subsystem attached to the event.
     * @return the subsystem that was recently added to the {@link SubsystemManager}.
     */
    public Subsystem getSubsystem()
    {
        return this.m_Subsystem;
    }
}
