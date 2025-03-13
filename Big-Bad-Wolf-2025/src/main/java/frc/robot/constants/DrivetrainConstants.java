package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DrivetrainConstants 
{
    public static Pose2d[] RED_CENTER_ALIGNMENTS = 
    {
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 1
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 2
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 3
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 4
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 5
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))  // 6 
    };

    public static Pose2d[] RED_LEFT_ALIGNMENTS = 
    {
        new Pose2d(11.6775, 4.1806, Rotation2d.fromDegrees(3.62)), // 1
        new Pose2d(12.515, 5.298, Rotation2d.fromDegrees(-57.031)), // 2
        new Pose2d(13.9055, 5.1241, Rotation2d.fromDegrees(-117.15)), // 3
        new Pose2d(14.438, 3.8534, Rotation2d.fromDegrees(-177.109)), // 4
        new Pose2d(13.6126, 2.7547, Rotation2d.fromDegrees(123.3357)), // 5
        new Pose2d(12.2162, 2.9196, Rotation2d.fromDegrees(62.457))  // 6 
    };

    public static Pose2d[] RED_RIGHT_ALIGNMENTS =
    {
        new Pose2d(11.6852, 3.8255, Rotation2d.fromDegrees(7.061)), // 1
        new Pose2d(12.202, 5.1099, Rotation2d.fromDegrees(-52.049)), // 2
        new Pose2d(13.573, 5.3125, Rotation2d.fromDegrees(-113.04)), // 3
        new Pose2d(14.4367, 4.2321, Rotation2d.fromDegrees(-172.337)), // 4
        new Pose2d(13.9304, 2.9429, Rotation2d.fromDegrees(127.817)), // 5
        new Pose2d(12.5739, 2.71538, Rotation2d.fromDegrees(68.597))  // 6 
    };

    public static Pose2d[] BLUE_CENTER_ALIGNMENTS = 
    {
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 1
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 2
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 3
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 4
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), // 5
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))  // 6 
    };

    public static Pose2d[] BLUE_LEFT_ALIGNMENTS = 
    {
        new Pose2d(5.8676, 3.8377, Rotation2d.fromDegrees(-177.372)), // 1
        new Pose2d(5.0277, 2.7457, Rotation2d.fromDegrees(122.435)), // 2
        new Pose2d(3.6498, 2.9169, Rotation2d.fromDegrees(63.016)), // 3
        new Pose2d(3.1139, 4.2322, Rotation2d.fromDegrees(1.873)), // 4
        new Pose2d(3.9715, 5.3166, Rotation2d.fromDegrees(-57.771)), // 5
        new Pose2d(5.3302, 5.1324, Rotation2d.fromDegrees(-116.487))  // 6 
    };

    public static Pose2d[] BLUE_RIGHT_ALIGNMENTS =
    {
        new Pose2d(5.866, 4.2277, Rotation2d.fromDegrees(-172.806)), // 1
        new Pose2d(5.352, 2.9334, Rotation2d.fromDegrees(127.567)), // 2
        new Pose2d(3.9863, 2.7228, Rotation2d.fromDegrees(66.934)), // 3
        new Pose2d(3.1134, 3.8326, Rotation2d.fromDegrees(7.867)), // 4
        new Pose2d(3.631, 5.1179, Rotation2d.fromDegrees(-53.098)), // 5
        new Pose2d(4.9986, 5.3219, Rotation2d.fromDegrees(-112.247))  // 6 
    };
}
