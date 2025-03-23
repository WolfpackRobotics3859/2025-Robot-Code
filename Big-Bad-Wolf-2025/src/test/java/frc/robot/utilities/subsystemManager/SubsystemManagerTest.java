package frc.robot.utilities.subsystemManager;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.SubsystemManager;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SubsystemManagerTest
{
    private SubsystemManager UUT;

    @BeforeEach
    void setup()
    {
        UUT = new SubsystemManager();
    }

    @Test
    void addSubsystemToManagerAndRetrieve()
    {
        Wrist intake = new Wrist();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.getSubsystemOfType(Wrist.class).get() == intake);
    }

    @Test
    void attemptToAddTwoSubsystems()
    {
        Wrist intake = new Wrist();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isPresent());
        assertTrue(UUT.removeSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Wrist.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isEmpty());
        assertFalse(UUT.addSubsystem(new Wrist()));
        assertTrue(UUT.removeSubsystem(intake));
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Wrist.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
    }

    @Test
    void attemptToRemoveASubsystem()
    {
        Wrist intake = new Wrist();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Wrist.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
        assertFalse(UUT.addSubsystem(new Wrist()));
    }

    @Test
    void testSubscriber()
    {
        TestSubsystemAddedListener subscriber = new TestSubsystemAddedListener();
        UUT.subscribeSubsystemAdded(subscriber);
        assertTrue(UUT.addSubsystem(new Elevator()));
        assertTrue(subscriber.subsystem.getClass() == Elevator.class);
        assertTrue(UUT.addSubsystem(new Wrist()));
        assertTrue(subscriber.subsystem.getClass() == Wrist.class);
    }
}

class TestSubsystemAddedListener implements SubsystemAddedListener
{
    public Subsystem subsystem;

    @Override
    public void onSubsystemAddedEvent(SubsystemAddedEvent event) {
        this.subsystem = event.getSubsystem();
    }

}
