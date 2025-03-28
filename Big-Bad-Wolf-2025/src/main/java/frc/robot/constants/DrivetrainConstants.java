package frc.robot.constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
        new Pose2d(11.6411, 4.1927, new Rotation2d(0.0192)), // 1
        new Pose2d(12.5106, 5.3501, new Rotation2d(-1.0089)), // 2
        new Pose2d(13.9130, 5.1462, new Rotation2d(-2.0602)), // 3
        new Pose2d(14.4761, 3.8383, new Rotation2d(-3.1078)), // 4
        new Pose2d(13.6073, 2.7019, new Rotation2d(2.1216)), // 5
        new Pose2d(12.2590, 2.8357, new Rotation2d(1.0835))  // 6 
    };

    public static Pose2d[] RED_RIGHT_ALIGNMENTS =
    {
        new Pose2d(11.6376, 3.8101, new Rotation2d(0.1349)), // 1
        new Pose2d(12.1613, 5.1443,new Rotation2d(-0.9318)), // 2
        new Pose2d(13.5839, 5.3569, new Rotation2d(-1.9752)), // 3
        new Pose2d(14.4785, 4.2318, new Rotation2d(-3.0317)), // 4
        new Pose2d(13.9674, 2.9187, new Rotation2d(2.2138)), // 5
        new Pose2d(12.5394, 2.6933, new Rotation2d(1.1639))  // 6 
    };

    // public static Pose2d[] RED_LEFT_ALIGNMENTS = 
    // {
    //     new Pose2d(11.6775, 4.1806, Rotation2d.fromDegrees(3.62)), // 1
    //     new Pose2d(12.515, 5.298, Rotation2d.fromDegrees(-57.031)), // 2
    //     new Pose2d(13.919, 5.164, Rotation2d.fromDegrees(-117.00)), // 3
    //     new Pose2d(14.438, 3.8534, Rotation2d.fromDegrees(-177.109)), // 4
    //     new Pose2d(13.6126, 2.7547, Rotation2d.fromDegrees(123.3357)), // 5
    //     new Pose2d(12.2162, 2.9196, Rotation2d.fromDegrees(62.457))  // 6 
    // };

    // public static Pose2d[] RED_RIGHT_ALIGNMENTS =
    // {
    //     new Pose2d(11.6852, 3.8255, Rotation2d.fromDegrees(7.061)), // 1
    //     new Pose2d(12.202, 5.1099, Rotation2d.fromDegrees(-52.049)), // 2
    //     new Pose2d(13.58, 5.32, Rotation2d.fromDegrees(-120)), // 3
    //     new Pose2d(14.4367, 4.2321, Rotation2d.fromDegrees(-172.337)), // 4
    //     new Pose2d(13.9304, 2.9429, Rotation2d.fromDegrees(127.817)), // 5
    //     new Pose2d(12.5739, 2.71538, Rotation2d.fromDegrees(68.597))  // 6 
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
        new Pose2d(5.9075, 3.8526, new Rotation2d(-3.1122)), // 1
        new Pose2d(5.0364, 2.7044, new Rotation2d(2.1396)), // 2
        new Pose2d(3.6183, 2.8929, new Rotation2d(1.0857)), // 3
        new Pose2d(3.071, 4.1822, new Rotation2d(0.0406)), // 4
        new Pose2d(3.9407, 5.3384, new Rotation2d(-1.0301)), // 5
        new Pose2d(5.1622, 5.456, new Rotation2d(-2.0262))  // 6 
    };

    public static Pose2d[] BLUE_RIGHT_ALIGNMENTS =
    {
        new Pose2d(5.8924, 4.2067, new Rotation2d(-3.0188)), // 1
        new Pose2d(5.3779, 2.9192, new Rotation2d(2.1986)), // 2
        new Pose2d(3.9742, 2.688, new Rotation2d(1.1524)), // 3
        new Pose2d(3.0867, 3.8162, new Rotation2d(0.0760)), // 4
        new Pose2d(3.6056, 5.1271, new Rotation2d(-0.9183)), // 5
        new Pose2d(5.0189, 5.3573, new Rotation2d(-1.9701))  // 6 
    };

    // public static Pose2d[] BLUE_LEFT_ALIGNMENTS = 
    // {
    //     new Pose2d(5.8676, 3.8377, Rotation2d.fromDegrees(-177.372)), // 1
    //     new Pose2d(5.0277, 2.7457, Rotation2d.fromDegrees(122.435)), // 2
    //     new Pose2d(3.6498, 2.9169, Rotation2d.fromDegrees(63.016)), // 3
    //     new Pose2d(3.1139, 4.2322, Rotation2d.fromDegrees(1.873)), // 4
    //     new Pose2d(3.9715, 5.3166, Rotation2d.fromDegrees(-57.771)), // 5
    //     new Pose2d(5.3302, 5.1324, Rotation2d.fromDegrees(-116.487))  // 6 
    // };

    // public static Pose2d[] BLUE_RIGHT_ALIGNMENTS =
    // {
    //     new Pose2d(5.866, 4.2277, Rotation2d.fromDegrees(-172.806)), // 1
    //     new Pose2d(5.352, 2.9334, Rotation2d.fromDegrees(127.567)), // 2
    //     new Pose2d(3.9863, 2.7228, Rotation2d.fromDegrees(66.934)), // 3
    //     new Pose2d(3.1134, 3.8326, Rotation2d.fromDegrees(7.867)), // 4
    //     new Pose2d(3.631, 5.1179, Rotation2d.fromDegrees(-53.098)), // 5
    //     new Pose2d(4.9986, 5.3219, Rotation2d.fromDegrees(-112.247))  // 6 
    // };

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
