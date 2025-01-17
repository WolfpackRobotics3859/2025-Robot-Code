package frc.robot.utilities;

import java.util.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.subsystemManager.SubsystemAddedEvent;
import frc.robot.utilities.subsystemManager.SubsystemAddedListener;

/*
 * Handles the storage of subsystems while notifying subscribers of new additions.
 */
public class SubsystemManager 
{
    private List<Subsystem> m_Subsystems = new ArrayList<>();

    private List<SubsystemAddedListener> m_SubsystemAddedListeners = new ArrayList<>();

    /**
     * Adds a subsystem to the manager.
     * @param subsystem the subsystem to add.
     * @return true when successfully added or false when a subsystem of that type already exists within the manager.
     */
    public boolean addSubsystem(Subsystem subsystem)
    {
        if(this.getSubsystemOfType(subsystem.getClass()).isPresent())
        {
            return false;
        }
        m_Subsystems.add(subsystem);
        this.notifySubsystemAddedListeners(new SubsystemAddedEvent(subsystem));
        CommandScheduler.getInstance().registerSubsystem(subsystem);
        return true;
    }

    /**
     * Removes a subsystem from the manager.
     * @param subsystem the subsystem to remove.
     * @return true if successfully removed or false otherwise.
     */
    public boolean removeSubsystem(Subsystem subsystem)
    {
        return m_Subsystems.remove(subsystem);
    }

    /**
     * Attempts to retrieve a subsystem of a given type from the manager.
     * @param <T> the type of subsystem to retrieve.
     * @param type the class type to retrieve.
     * @return an {@link Optional} containing the subsystem if found, or an empty {@link Optional} otherwise.
     */
    public <T> Optional<T> getSubsystemOfType(Class<T> type)
    {
        for (Subsystem subsystemInList : m_Subsystems)
        {
            if (type.isInstance(subsystemInList))
            {
                return Optional.of(type.cast(subsystemInList));
            }
        }
        return Optional.empty();
    }

    /**
     * Subscribes a listener to added subsystem events.
     * @param listener the listener to add to the subscription list.
     */
    public void subscribeSubsystemAdded(SubsystemAddedListener listener)
    {
        m_SubsystemAddedListeners.add(listener);
    }

    /**
     * Unsubscribes a listerner to added subsystem events.
     * @param listener the listener to remove from the subscription list.
     */
    public void unsubscribeSubsystemAdded(SubsystemAddedListener listener)
    {
        m_SubsystemAddedListeners.remove(listener);
    }
 
    /**
     * Notifies listeners of an added subsystem event.
     * @param event the event object to notify subscribers with.
     */
    private void notifySubsystemAddedListeners(SubsystemAddedEvent event)
    {
        for (SubsystemAddedListener listener : m_SubsystemAddedListeners)
        {
            listener.onSubsystemAddedEvent(event);
        }
    }
}
