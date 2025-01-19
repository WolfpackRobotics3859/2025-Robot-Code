package frc.robot.utilities.subsystemManager;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.AlgaeCleaner;
import frc.robot.subsystems.AlgaeIntake;
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
        AlgaeIntake intake = new AlgaeIntake();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
    }

    @Test
    void attemptToAddTwoSubsystems()
    {
        AlgaeIntake intake = new AlgaeIntake();
        AlgaeCleaner cleaner = new AlgaeCleaner();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(AlgaeCleaner.class).isPresent());
        assertTrue(UUT.removeSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(AlgaeCleaner.class).isEmpty());
        assertFalse(UUT.addSubsystem(new AlgaeIntake()));
        assertTrue(UUT.removeSubsystem(intake));
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(AlgaeCleaner.class).get() == cleaner);
    }

    @Test
    void attemptToRemoveASubsystem()
    {
        AlgaeIntake intake = new AlgaeIntake();
        AlgaeCleaner cleaner = new AlgaeCleaner();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(cleaner));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(AlgaeCleaner.class).get() == cleaner);
        assertFalse(UUT.addSubsystem(new AlgaeIntake()));
    }

    @Test
    void testSubscriber()
    {
        TestSubsystemAddedListener subscriber = new TestSubsystemAddedListener();
        UUT.subscribeSubsystemAdded(subscriber);
        assertTrue(UUT.addSubsystem(new AlgaeCleaner()));
        assertTrue(subscriber.subsystem.getClass() == AlgaeCleaner.class);
        assertTrue(UUT.addSubsystem(new AlgaeIntake()));
        assertTrue(subscriber.subsystem.getClass() == AlgaeIntake.class);
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
