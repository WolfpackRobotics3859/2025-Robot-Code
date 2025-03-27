package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldCalibrator {
    
    private CommandSwerveDrivetrain commandSwerveDrivetrain;

    public FieldCalibrator(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    public Pose2d getPose2d() {
        return this.commandSwerveDrivetrain.getState().Pose;
    }
}
