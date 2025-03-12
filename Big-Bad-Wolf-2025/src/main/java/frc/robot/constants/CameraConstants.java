package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraConstants 
{
    public static String[] CAMERA_PIPELINES =
    {
        "FORWARD_CAM",
      //  "REAR_CAM"
    };

    public static Transform3d[] ROBOT_TO_CAM_TRANFORMS =
    {
        new Transform3d(0.0529080684974, -0.228599418, 0.6483841884469, new Rotation3d(0, 0.610865, 0.349066)), // Forward Cam
      //  new Transform3d(0.0529080684974, -0.228599418, 0.6483841884469, new Rotation3d(0, 0.610865, 0.349066)) // Rear Cam
    };
}
