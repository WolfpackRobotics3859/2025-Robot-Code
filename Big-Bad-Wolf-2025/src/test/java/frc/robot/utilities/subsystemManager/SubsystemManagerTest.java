package frc.robot.utilities.subsystemManager;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.Shooter;
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
        Shooter intake = new Shooter();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == intake);
    }

    @Test
    void attemptToAddTwoSubsystems()
    {
        Shooter intake = new Shooter();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isPresent());
        assertTrue(UUT.removeSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isEmpty());
        assertFalse(UUT.addSubsystem(new Shooter()));
        assertTrue(UUT.removeSubsystem(intake));
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
    }

    @Test
    void attemptToRemoveASubsystem()
    {
        Shooter intake = new Shooter();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
        assertFalse(UUT.addSubsystem(new Shooter()));
    }

    @Test
    void testSubscriber()
    {
        TestSubsystemAddedListener subscriber = new TestSubsystemAddedListener();
        UUT.subscribeSubsystemAdded(subscriber);
        assertTrue(UUT.addSubsystem(new Elevator()));
        assertTrue(subscriber.subsystem.getClass() == Elevator.class);
        assertTrue(UUT.addSubsystem(new Shooter()));
        assertTrue(subscriber.subsystem.getClass() == Shooter.class);
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
