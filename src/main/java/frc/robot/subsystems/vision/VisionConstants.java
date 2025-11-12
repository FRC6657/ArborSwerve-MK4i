package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.15;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static class CameraInfo {

    public String cameraName;
    public Transform3d robotToCamera;
    public Rotation2d diagFOV;
    public int[] cameraRes;

    public CameraInfo(
        String cameraName, Transform3d robotToCamera, Rotation2d diagFOV, int[] cameraRes) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.diagFOV = diagFOV;
      this.cameraRes = cameraRes;
    }
  }

  public static CameraInfo ExampleCameraInfo1 =
      new CameraInfo(
          "ExampleCamera",
          new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});

  public static CameraInfo ExampleCameraInfo2 =
      new CameraInfo(
          "ExampleCamera",
          new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
          Rotation2d.fromDegrees(80),
          new int[] {1280, 800});
}
