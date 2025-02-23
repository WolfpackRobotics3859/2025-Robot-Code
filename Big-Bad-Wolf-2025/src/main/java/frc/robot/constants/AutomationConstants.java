package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AutomationConstants 
{
    public static final String FORWARD_CAMERA_NAME = "FORWARD_CAM";
    public static final Transform3d FORWARD_CAMERA_TO_ROBOT = new Transform3d(0.0529080684974, -0.228599418, 0.6483841884469, new Rotation3d(0, 0.610865, 0.349066));
}
