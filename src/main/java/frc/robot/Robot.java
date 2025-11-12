// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.DrivebaseConstants;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_CTRE;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameras;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.DriveToPose;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

@SuppressWarnings({"unused", "resource"})
public class Robot extends LoggedRobot {

  // Controllers
  private CommandXboxController driver = new CommandXboxController(0);

  // Subsystems
  private final ApriltagCameras cameras;
  private final Drivebase drivebase;

  // Superstructure
  private final Superstructure superstructure;

  // Auto Stuff
  private final AutoFactory autoFactory;
  private LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  public Robot() {

    // Get rid  of annoying warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);

    drivebase =
        new Drivebase(
            RobotBase.isReal() ? new GyroIO_CTRE() : new GyroIO() {},
            RobotBase.isReal()
                ? new ModuleIO[] {
                  new ModuleIO_Real(DrivebaseConstants.kFrontLeftModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kFrontRightModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kBackLeftModuleConstants),
                  new ModuleIO_Real(DrivebaseConstants.kBackRightModuleConstants)
                }
                : new ModuleIO[] {
                  new ModuleIO_Sim(DrivebaseConstants.kFrontLeftModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kFrontRightModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kBackLeftModuleConstants),
                  new ModuleIO_Sim(DrivebaseConstants.kBackRightModuleConstants)
                });

    cameras =
        new ApriltagCameras(
            drivebase::addVisionMeasurement,
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.ExampleCameraInfo1)
                : new ApriltagCameraIO_Sim(VisionConstants.ExampleCameraInfo1, drivebase::getPose),
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.ExampleCameraInfo2)
                : new ApriltagCameraIO_Sim(VisionConstants.ExampleCameraInfo1, drivebase::getPose));

    superstructure = new Superstructure(drivebase);

    driver
        .a()
        .whileTrue(
            new DriveToPose(
                drivebase,
                () -> new Pose2d(0, 0, new Rotation2d()),
                Units.inchesToMeters(1),
                Units.inchesToMeters(1),
                Units.degreesToRadians(3),
                Units.degreesToRadians(3),
                new Constraints(1, 1),
                new Constraints(Units.degreesToRadians(90), Units.degreesToRadians(90))));

    autoFactory =
        new AutoFactory(
            drivebase::getPose, drivebase::resetPose, drivebase::followTrajectory, true, drivebase);

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
  }

  public static boolean replay = false;

  @Override
  public void robotInit() {
    Logger.recordMetadata("Arborbotics", "RobotName");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      if (!replay) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
      }
    }

    drivebase.setDefaultCommand(
        drivebase.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * 0.25,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * DrivebaseConstants.kMaxLinearSpeed
                        * 0.25,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * DrivebaseConstants.kMaxAngularSpeed
                        * 0.25)));

    System.out.println("Linear Speed: " + DrivebaseConstants.kMaxLinearSpeed);
    System.out.println("Angular Speed: " + DrivebaseConstants.kMaxAngularSpeed);

    autoChooser.addOption(
        "TestAuto", Commands.run(() -> drivebase.drive(new ChassisSpeeds(1, 0, 0))));

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autoChooser.get().schedule();
  }

  @Override
  public void teleopInit() {
    if (autoChooser.get() != null) {
      autoChooser.get().cancel();
    }
  }
}
