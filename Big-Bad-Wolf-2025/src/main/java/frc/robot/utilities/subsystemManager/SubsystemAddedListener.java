package frc.robot.utilities.subsystemManager;

/**
 * Enables the listening of subsystem added events in the {@link SubsystemManager}.
 */
public interface SubsystemAddedListener 
{
    /**
     * The method ran when a subsystem is added to the manager.
     * @param event the event object.
     */
    void onSubsystemAddedEvent(SubsystemAddedEvent event);    
}
