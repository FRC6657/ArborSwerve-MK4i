package frc.robot.subsystems.drivebase;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivebase extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules; // FL FR BL BR

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  static final Lock odometryLock = new ReentrantLock();

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public Drivebase(GyroIO gyroIO, ModuleIO[] moduleIOs) {

    this.modules = new Module[moduleIOs.length];

    this.kinematics = new SwerveDriveKinematics(DrivebaseConstants.kModuleTranslations);
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());

    this.setpointGenerator =
        new SwerveSetpointGenerator(
            DrivebaseConstants.kPathPlannerConfig, DrivebaseConstants.kMaxAzimuthSpeed);
    previousSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(),
              new SwerveModuleState(),
              new SwerveModuleState(),
              new SwerveModuleState()
            },
            DriveFeedforwards.zeros(moduleIOs.length));

    this.gyroIO = gyroIO;

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }

    // Enable Wrapping
    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    PhoenixOdometryThread.getInstance().start();
  }

  /**
   * @return The current pose of the robot
   */
  @AutoLogOutput(key = "Swerve/Pose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @param pose The new pose of the robot
   */
  public void resetPose(Pose2d pose) {

    var yaw =
        (RobotBase.isSimulation() && !Robot.replay) ? pose.getRotation() : gyroInputs.yawPosition;
    poseEstimator.resetPosition(
        yaw,
        Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new),
        pose);
  }

  /**
   * @return The velocity of the robot in robot relative coordinates
   */
  public ChassisSpeeds getVelocityRobotRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * @return The velocity of the robot in field relative coordinates
   */
  public ChassisSpeeds getVelocityFieldRelative() {
    var speeds =
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());
    return speeds;
  }

  public void drive(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds The desired robot relative speeds
   */
  public void drive(ChassisSpeeds speeds, boolean openLoop) {

    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);

    for (int i = 0; i < previousSetpoint.moduleStates().length; i++) {
      modules[i].runSetpoint(previousSetpoint.moduleStates()[i], openLoop); // Run setpoints
    }

    Logger.recordOutput("Swerve/Module Setpoints", previousSetpoint.moduleStates());
    Logger.recordOutput(
        "Swerve/Module States",
        Arrays.stream(modules).map(m -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /**
   * Runs the drivetrain at the given robot relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain at the given robot relative speeds
   */
  public Command driveVelocity(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> drive(speeds.get()));
  }

  /**
   * Runs the drivetrain at the given field relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain at the given field relative speeds
   */
  public Command driveVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.driveVelocity(
        () -> {
          var speed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation());
          return speed;
        });
  }

  /**
   * Runs the drivetrain for teleop with field relative speeds
   *
   * @param speeds The desired field relative speeds
   * @return The command to run the drivetrain for teleop with field relative speeds
   */
  public Command driveTeleop(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          var speed =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose().getRotation().minus(Rotation2d.fromDegrees(180)));
          this.drive(speed, false);
        });
  }

  PIDController choreoXController = DrivebaseConstants.kChoreoXController;
  PIDController choreoYController = DrivebaseConstants.kChoreoYController;
  PIDController choreoThetaController = DrivebaseConstants.kChoreoThetaController;

  public void followTrajectory(SwerveSample sample) {

    Logger.recordOutput("Choreo/DesiredPose", sample.getPose());

    double xFF = sample.vx;
    double yFF = sample.vy;
    double rotationFF = sample.omega;

    double xFeedback = choreoXController.calculate(getPose().getX(), sample.x);
    double yFeedback = choreoYController.calculate(getPose().getY(), sample.y);
    double rotationFeedback =
        choreoThetaController.calculate(getPose().getRotation().getRadians(), sample.heading);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            getPose().getRotation());

    drive(out);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (RobotBase.isReal() || Robot.replay) {
      poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }
  }

  /**
   * Runs a characterization routine that will determine the "real" wheel radius of the swerve
   * modules It does this by rotating the robot and measuring the distance traveled by the wheels.
   * It then compares the expected distance based on the gyro reading to the observed distance to
   * back calculate the "real" wheel radius.
   */
  public Command wheelRadiusCharacterization() {

    double driveRadius =
        DrivebaseConstants.kModuleTranslations[0].getNorm(); // Radius of drive wheel circle
    ChassisSpeeds speeds =
        new ChassisSpeeds(0, 0, Units.rotationsToRadians(0.2)); // Speed to rotate at

    return Commands.runOnce(
            () -> {
              // Reset Gyro and Zero Wheel Encoders
              gyroIO.setYaw(new Rotation2d());
              for (var module : modules) {
                module.resetDriveEncoder();
              }
            })
        .andThen(
            // Turn for 5 seconds at the desired speed to collect gyro/encoder data
            Commands.sequence(
                Commands.waitSeconds(1),
                Commands.runEnd(() -> this.drive(speeds), () -> this.drive(new ChassisSpeeds()))
                    .raceWith(Commands.waitSeconds(5))))
        .andThen(
            // Process collected data
            this.runOnce(
                () -> {

                  // Wheels should have all moved more or less the same distance, average them just
                  // incase.
                  double avgWheelRoations = 0;
                  for (var module : modules) {
                    avgWheelRoations +=
                        Math.abs(
                            module.getPosition().distanceMeters
                                / (Units.inchesToMeters(DrivebaseConstants.kDriveWheelDiameter)
                                    * Math.PI));
                  }
                  avgWheelRoations /= 4;

                  // Compare the expected distance to the observed distance to back calculate the
                  // "real" wheel radius.
                  double expectedDistance =
                      (driveRadius * 2 * Math.PI) * Units.degreesToRotations(gyroInputs.yaw);
                  double observedRadius = expectedDistance / (avgWheelRoations * 2 * Math.PI);

                  // Log the calculated radius
                  Logger.recordOutput("Characterization/WheelRadius", observedRadius);
                }));
  }

  @Override
  public void periodic() {

    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (Module module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    if (RobotBase.isReal() || Robot.replay) {
      double[] sampleTimestamps = modules[0].getOdometryTimestamps();
      for (int i = 0; i < sampleTimestamps.length; i++) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
          try {
            poseEstimator.updateWithTime(
                sampleTimestamps[i], gyroInputs.yawPositions[i], modulePositions);
          } catch (Exception e) {
            Logger.recordOutput("Errors", "Pose Estimator failed to update: " + e.getMessage());
          }
        }
      }
    } else {
      var simHeading = getPose().getRotation();
      var gyroDelta =
          new Rotation2d(
                  kinematics.toChassisSpeeds(
                          Arrays.stream(modules)
                              .map(m -> m.getState())
                              .toArray(SwerveModuleState[]::new))
                      .omegaRadiansPerSecond)
              .times(0.02);

      simHeading = simHeading.plus(gyroDelta);

      poseEstimator.update(
          simHeading,
          Arrays.stream(modules).map(m -> m.getPosition()).toArray(SwerveModulePosition[]::new));
    }
  }
}
