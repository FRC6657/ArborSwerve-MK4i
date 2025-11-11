package frc.robot.util;

// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.Drivebase;
import frc.robot.subsystems.drivebase.DrivebaseConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final Drivebase drivebase;
  private final Supplier<Pose2d> target;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController = new PIDController(3.75, 0.0, 0.0, 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(4.375, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private Supplier<Pose2d> robot;
  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(
      Drivebase drivebase,
      Supplier<Pose2d> target,
      double positionTolerance,
      double linearVelocityTolerance,
      double rotationTolerance,
      double angularVelocityTolerance,
      Constraints positionConstraints,
      Constraints rotationConstraints) {
    this.drivebase = drivebase;
    this.target = target;

    robot = drivebase::getPose;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(positionTolerance, linearVelocityTolerance);
    thetaController.setTolerance(rotationTolerance, angularVelocityTolerance);

    driveProfile = new TrapezoidProfile(positionConstraints);
    thetaController.setConstraints(rotationConstraints);

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = drivebase.getVelocityFieldRelative();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {

    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(Units.rotationsToRadians(1), Units.rotationsToRadians(1)));

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler = MathUtil.clamp((driveErrorAbs - 0.01) / (0.05 - 0.01), 0.0, 1.0);
    double thetaFFScaler =
        MathUtil.clamp(
            (Units.radiansToDegrees(thetaErrorAbs) - Units.degreesToRadians(2))
                / (Units.degreesToRadians(4) - Units.degreesToRadians(2)),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm() <= 0.01 // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, -0.5);
    State driveSetpoint =
        driveProfile.calculate(
            0.02,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = MathUtil.clamp(linearFF.get().getNorm() * 3.0, 0.0, 1.0);
    final double thetaS = MathUtil.clamp(Math.abs(omegaFF.getAsDouble()) * 3.0, 0.0, 1.0);
    driveVelocity =
        driveVelocity.interpolate(
            linearFF.get().times(DrivebaseConstants.kMaxLinearSpeed), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DrivebaseConstants.kMaxAngularSpeed, thetaS);
    // Reset profiles if enough input
    ChassisSpeeds fieldVelocity = drivebase.getVelocityFieldRelative();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    if (linearS >= 0.2) {
      lastSetpointTranslation = currentPose.getTranslation();
      lastSetpointVelocity = linearFieldVelocity;
    }
    if (thetaS >= 0.1) {
      thetaController.reset(
          currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    }

    // Command speeds
    drivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/Data/Distance Measured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/Data/Distance Setpoint", driveSetpoint.position);
    Logger.recordOutput(
        "DriveToPose/Data/Velocity Measured",
        -linearFieldVelocity
                .toVector()
                .dot(targetPose.getTranslation().minus(currentPose.getTranslation()).toVector())
            / driveErrorAbs);
    Logger.recordOutput("DriveToPose/Data/Velocity Setpoint", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/Data/Theta Measured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/Data/Theta Setpoint", thetaController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/State/Target Pose", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds());
    Logger.recordOutput("DriveToPose/State/Target Pose", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/State/At Position", false);
    Logger.recordOutput("DriveToPose/State/At Rotation", false);
    Logger.recordOutput("DriveToPose/State/At Rotational Velocity", false);
  }

  @Override
  public boolean isFinished() {

    double positionTolerance = driveController.getErrorTolerance();
    double rotationTolerance = thetaController.getPositionTolerance();
    double angularVelocityTolerance = thetaController.getVelocityTolerance();

    boolean atPositionTolerance = driveErrorAbs < positionTolerance;
    boolean atRotationTolerance = thetaErrorAbs < rotationTolerance;
    boolean atAngularVelocityTolerance =
        thetaController.getVelocityError() < angularVelocityTolerance;

    Logger.recordOutput("DriveToPose/State/At Position", atPositionTolerance);
    Logger.recordOutput("DriveToPose/State/At Rotation", atRotationTolerance);
    Logger.recordOutput("DriveToPose/State/At Rotational Velocity", atAngularVelocityTolerance);

    return atPositionTolerance && atRotationTolerance && atAngularVelocityTolerance;
  }
}
