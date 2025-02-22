package frc.robot.constants;

import frc.robot.subsystems.photonUtilities.Camera;
import frc.robot.subsystems.photonUtilities.Camera.CAMERA_PLACEMENT;

public class PhotonConstants {
// Camera Objects
  public static Camera frontCamera = new Camera("CameraRight1", CAMERA_PLACEMENT.FRONT);
  public static Camera backCamera = new Camera("BackCamera", CAMERA_PLACEMENT.BACK);
  public static Camera leftCamera = new Camera("LeftCamera", CAMERA_PLACEMENT.LEFT);
  public static Camera rightCamera = new Camera("RightCamera", CAMERA_PLACEMENT.RIGHT);

  // public static Camera m_LegacyFrontCamera = new Camera("DriverCamera", CAMERA_PLACEMENT.FRONT);
  // public static Camera m_LegacyLeftCamera = new Camera("CameraLeft1", CAMERA_PLACEMENT.LEFT);
  // public static Camera m_LegacyRightCamera = new Camera("CameraRight1", CAMERA_PLACEMENT.RIGHT);
}
