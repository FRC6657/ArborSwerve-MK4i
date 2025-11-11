package frc.robot.subsystems.drivebase;

import com.reduxrobotics.frames.DoubleFrame;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivebase.DrivebaseConstants.CAN;
import java.util.Queue;

/** Currently Untested, */
public class GyroIO_Redux implements GyroIO {

  private final Canandgyro gyro;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIO_Redux() {

    gyro = new Canandgyro(CAN.Gyro.id);

    gyro.setSettings(
        new CanandgyroSettings()
            .setAngularPositionFramePeriod(1d / DrivebaseConstants.kOdometryFrequency));

    gyro.setYaw(0); // Zero Gyro

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    DoubleFrame<Double> yaw = gyro.getYawFrame();

    // Assign Inputs
    inputs.yawPosition = Rotation2d.fromRotations(yaw.getValue()); // Normalized Yaw
    inputs.yaw = Units.rotationsToDegrees(yaw.getValue()); // Raw Yaw
    inputs.yawVelocityRadPerSec =
        Units.rotationsToRadians(gyro.getAngularVelocityYaw()); // Yaw Velocity
    inputs.yawTimestamp = yaw.getTimestamp();

    inputs.yawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.yawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  /** Set the yaw of the gyro */
  @Override
  public void setYaw(Rotation2d yaw) {
    gyro.setYaw(yaw.getRotations());
  }
}
