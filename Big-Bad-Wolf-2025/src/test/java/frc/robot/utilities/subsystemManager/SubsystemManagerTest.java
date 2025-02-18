package frc.robot.utilities.subsystemManager;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.Intake;
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
        Intake intake = new Intake();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.getSubsystemOfType(Intake.class).get() == intake);
    }

    @Test
    void attemptToAddTwoSubsystems()
    {
        Intake intake = new Intake();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isPresent());
        assertTrue(UUT.removeSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Intake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).isEmpty());
        assertFalse(UUT.addSubsystem(new Intake()));
        assertTrue(UUT.removeSubsystem(intake));
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Intake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
    }

    @Test
    void attemptToRemoveASubsystem()
    {
        Intake intake = new Intake();
        Elevator cleaner = new Elevator();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(Intake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Elevator.class).get() == cleaner);
        assertFalse(UUT.addSubsystem(new Intake()));
    }

    @Test
    void testSubscriber()
    {
        TestSubsystemAddedListener subscriber = new TestSubsystemAddedListener();
        UUT.subscribeSubsystemAdded(subscriber);
        assertTrue(UUT.addSubsystem(new Elevator()));
        assertTrue(subscriber.subsystem.getClass() == Elevator.class);
        assertTrue(UUT.addSubsystem(new Intake()));
        assertTrue(subscriber.subsystem.getClass() == Intake.class);
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
