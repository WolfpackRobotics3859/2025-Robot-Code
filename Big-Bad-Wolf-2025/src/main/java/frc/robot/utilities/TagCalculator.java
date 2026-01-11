package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;

public class TagCalculator 
{
    private AprilTagFieldLayout m_TagLayout;

    public TagCalculator()
    {
        m_TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }


    private void LoadField()
    {

    }

}
