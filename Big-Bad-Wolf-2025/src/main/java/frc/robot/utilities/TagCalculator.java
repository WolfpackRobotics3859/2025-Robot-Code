package frc.robot.utilities;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class TagCalculator 
{
    private AprilTagFieldLayout m_TagLayout;

    public TagCalculator()
    {
        m_TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

}
