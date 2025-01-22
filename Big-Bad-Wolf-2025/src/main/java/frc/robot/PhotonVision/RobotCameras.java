package frc.robot.PhotonVision;

import frc.robot.PhotonVision.Camera.CAMERA_PLACEMENT;

public class RobotCameras {
// Camera Objects
  public static Camera m_FrontCamera = new Camera("FrontCamera", CAMERA_PLACEMENT.FRONT);
  public static Camera m_BackCamera = new Camera("BackCamera", CAMERA_PLACEMENT.BACK);
  public static Camera m_LeftCamera = new Camera("LeftCamera", CAMERA_PLACEMENT.LEFT);
  public static Camera m_RightCamera = new Camera("RightCamera", CAMERA_PLACEMENT.RIGHT);

  public static Camera m_LegacyFrontCamera = new Camera("DriverCamera", CAMERA_PLACEMENT.FRONT);
  public static Camera m_LegacyLeftCamera = new Camera("CameraLeft1", CAMERA_PLACEMENT.LEFT);
  public static Camera m_LegacyRightCamera = new Camera("CameraRight1", CAMERA_PLACEMENT.RIGHT);
}
