package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldCalibrator
{
    
    private CommandSwerveDrivetrain commandSwerveDrivetrain;

    private Pose2d currentPose2d;

    public FieldCalibrator(CommandSwerveDrivetrain commandSwerveDrivetrain)
    {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }
    
    public Pose2d getPose()
    {
        this.currentPose2d = this.commandSwerveDrivetrain.getState().Pose;
        return currentPose2d;
    }
}
