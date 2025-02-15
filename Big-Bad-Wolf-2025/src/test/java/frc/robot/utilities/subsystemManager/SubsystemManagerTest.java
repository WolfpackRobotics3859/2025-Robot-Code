package frc.robot.utilities.subsystemManager;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;

import frc.robot.subsystems.Shooter;
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
        Shooter shooter = new Shooter();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(shooter));
        assertTrue(UUT.getSubsystemOfType(Shooter.class).isPresent());
        assertTrue(UUT.removeSubsystem(shooter));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Shooter.class).isEmpty());
        assertFalse(UUT.addSubsystem(new AlgaeIntake()));
        assertTrue(UUT.removeSubsystem(intake));
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(shooter));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == shooter);
    }

    @Test
    void attemptToRemoveASubsystem()
    {
        AlgaeIntake intake = new AlgaeIntake();
        Shooter shooter = new Shooter();
        assertTrue(UUT.addSubsystem(intake));
        assertTrue(UUT.addSubsystem(shooter));
        assertTrue(UUT.getSubsystemOfType(AlgaeIntake.class).get() == intake);
        assertTrue(UUT.getSubsystemOfType(Shooter.class).get() == shooter);
        assertFalse(UUT.addSubsystem(new AlgaeIntake()));
    }

    @Test
    void testSubscriber()
    {
        TestSubsystemAddedListener subscriber = new TestSubsystemAddedListener();
        UUT.subscribeSubsystemAdded(subscriber);
        assertTrue(UUT.addSubsystem(new Shooter()));
        assertTrue(subscriber.subsystem.getClass() == Shooter.class);
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
