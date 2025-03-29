package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DrivetrainConstants 
{
    public static Pose2d[] RED_CENTER_ALIGNMENTS = 
    {
        new Pose2d(11.749, 4.005, Rotation2d.fromDegrees(0)), // 1
        new Pose2d(12.412, 5.146, Rotation2d.fromDegrees(-60.898)), // 2
        new Pose2d(13.738, 5.136, Rotation2d.fromDegrees(-120.078)), // 3
        new Pose2d(14.381, 4.035, Rotation2d.fromDegrees(180)), // 4
        new Pose2d(13.728, 2.884, Rotation2d.fromDegrees(121.918)), // 5
        new Pose2d(12.412, 2.904, Rotation2d.fromDegrees(60.771))  // 6 
    };

    public static Pose2d[] RED_LEFT_ALIGNMENTS = 
    {
        new Pose2d(11.646, 4.222, new Rotation2d(0.045)), // 1
        new Pose2d(12.516, 5.343, new Rotation2d(-0.996)), // 2
        new Pose2d(13.923, 5.162, new Rotation2d(-2.049)), // 3 
        new Pose2d(14.473, 3.839, new Rotation2d(-3.085)), // 4
        new Pose2d(13.604, 2.707, new Rotation2d(2.604)), // 5
        new Pose2d(12.187, 2.896, new Rotation2d(1.101))  // 6 
    };

    public static Pose2d[] RED_RIGHT_ALIGNMENTS =
    {
        new Pose2d(11.648, 3.851, new Rotation2d(0.119)), // 1
        new Pose2d(12.204, 5.159,new Rotation2d(-0.927)), // 2
        new Pose2d(13.602, 5.347, new Rotation2d(-1.977)), // 3 
        new Pose2d(14.472, 4.217, new Rotation2d(-3.008)), // 4
        new Pose2d(13.929, 2.894, new Rotation2d(2.208)), // 5
        new Pose2d(12.505, 2.719, new Rotation2d(1.170))  // 6 
    };

    // public static Pose2d[] RED_LEFT_ALIGNMENTS = 
    // {
    //     new Pose2d(11.6384, 4.1981, new Rotation2d(0.0387)), // 1
    //     new Pose2d(12.4914, 5.3364, new Rotation2d(-1.0100)), // 2
    //     new Pose2d(13.9099, 5.1726, new Rotation2d(-2.0438)), // 3
    //     new Pose2d(14.4779, 3.8453, new Rotation2d(-3.1086)), // 4
    //     new Pose2d(13.6189, 2.7097, new Rotation2d(2.1411)), // 5
    //     new Pose2d(12.2047, 2.8825, new Rotation2d(1.0877))  // 6 
    // };

    // public static Pose2d[] RED_RIGHT_ALIGNMENTS =
    // {
    //     new Pose2d(11.6408, 3.8164, new Rotation2d(0.0936)), // 1
    //     new Pose2d(12.1273, 5.0967, new Rotation2d(-0.9292)), // 2
    //     new Pose2d(13.5690, 5.3686, new Rotation2d(-1.9993)), // 3
    //     new Pose2d(14.4785, 4.2318, new Rotation2d(-3.0317)), // 4
    //     new Pose2d(13.9747, 2.9172, new Rotation2d(2.2303)), // 5
    //     new Pose2d(12.8102, 2.6742, new Rotation2d(1.1241))  // 6 
    // };

    

    public static Pose2d[] RED_STAGING_POSES =
    {
        new Pose2d(11.411, 4.025, Rotation2d.fromDegrees(0)), // 1
        new Pose2d(12.210, 5.439, Rotation2d.fromDegrees(-60.101)), // 2
        new Pose2d(13.846, 5.459, Rotation2d.fromDegrees(-119.358)), // 3
        new Pose2d(14.702, 4.044, Rotation2d.fromDegrees(180)), // 4
        new Pose2d(13.951, 2.620, Rotation2d.fromDegrees(120.018)), // 5
        new Pose2d(12.248, 2.611, Rotation2d.fromDegrees(62.766))  // 6 
    };

    public static Pose2d[] BLUE_CENTER_ALIGNMENTS = 
    {
        new Pose2d(5.801,4.005, Rotation2d.fromDegrees(180.000)), // 1
        new Pose2d(5.168, 2.904, Rotation2d.fromDegrees(119.873)), // 2
        new Pose2d(3.841, 2.874, Rotation2d.fromDegrees(60.793)), // 3
        new Pose2d(3.198, 4.035, Rotation2d.fromDegrees(0)), // 4
        new Pose2d(3.841, 5.166, Rotation2d.fromDegrees(-57.720)), // 5
        new Pose2d(5.158, 5.156, Rotation2d.fromDegrees(-119.229))  // 6 
    };

    public static Pose2d[] BLUE_LEFT_ALIGNMENTS = 
    {
        new Pose2d(5.9105, 3.8580, new Rotation2d(-3.1110)), // 1
        new Pose2d(5.0433, 2.7087, new Rotation2d(2.1325)), // 2
        new Pose2d(3.6119, 2.8963, new Rotation2d(1.0707)), // 3
        new Pose2d(3.0689, 4.1881, new Rotation2d(0.0403)), // 4
        new Pose2d(3.9143, 5.3328, new Rotation2d(-1.0027)), // 5
        new Pose2d(5.1461, 5.4529, new Rotation2d(-2.0111))  // 6 
    };

    public static Pose2d[] BLUE_RIGHT_ALIGNMENTS =
    {
        new Pose2d(5.8339, 4.0699, new Rotation2d(-3.0899)), // 1
        new Pose2d(5.3681, 2.9003, new Rotation2d(2.2217)), // 2
        new Pose2d(3.9547, 2.6963, new Rotation2d(1.1752)), // 3
        new Pose2d(3.0673, 4.0466, new Rotation2d(0.0294)), // 4
        new Pose2d(3.5822, 5.1185, new Rotation2d(-0.9280)), // 5
        new Pose2d(4.9860, 5.3797, new Rotation2d(-1.9858))  // 6 
    };

    public static Pose2d[] BLUE_STAGING_POSES =
    {
        new Pose2d(6.115, 4.025, Rotation2d.fromDegrees(180)), // 1
        new Pose2d(5.253, 2.611, Rotation2d.fromDegrees(120)), // 2
        new Pose2d(3.665, 2.601, Rotation2d.fromDegrees(60)), // 3
        new Pose2d(2.890, 4.025, Rotation2d.fromDegrees(0)), // 4
        new Pose2d(3.686, 5.407, Rotation2d.fromDegrees(-60)), // 5
        new Pose2d(5.278, 5.428, Rotation2d.fromDegrees(-120))  // 6 
    };
}
