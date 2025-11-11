package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drivebase.DrivebaseConstants.ModuleConstants;

public class ModuleIO_Sim implements ModuleIO {

  private final ModuleConstants constants;

  private static final DCMotor driveMotor = DrivebaseConstants.kDriveMotor;
  private static final DCMotor turnMotor = DrivebaseConstants.kTurnMotor;

  private final TalonFX drive;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double turnAppliedVolts = 0.0;

  private final VelocityVoltage drivePID = new VelocityVoltage(0.0);
  private final VoltageOut driveOpenLoop = new VoltageOut(0);
  private final PIDController turnPID = new PIDController(100.0, 0.0, 0.0);

  public ModuleIO_Sim(ModuleConstants constants) {

    this.constants = constants;

    drive = new TalonFX(constants.driveID());
    drive.getConfigurator().apply(DrivebaseConstants.driveConfig);

    turnPID.enableContinuousInput(-0.5, 0.5);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.005, DrivebaseConstants.kDriveRatio),
            driveMotor,
            0,
            0);

    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotor, 0.0004, DrivebaseConstants.kTurnRatio),
            turnMotor,
            0,
            0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    var driveSimState = drive.getSimState();
    driveSimState.Orientation = ChassisReference.Clockwise_Positive;
    driveSim.setInput(driveSimState.getMotorVoltage());

    driveSim.update(0.02);
    turnSim.update(0.02);

    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60.0) * DrivebaseConstants.kDriveRatio);

    inputs.name = constants.name();

    inputs.drivePositionMeters =
        driveSim.getAngularPositionRotations() * (DrivebaseConstants.kDriveWheelDiameter * Math.PI);
    inputs.driveVelocityMetersPerSec =
        (driveSim.getAngularVelocityRPM() / 60)
            * (DrivebaseConstants.kDriveWheelDiameter * Math.PI);
    inputs.driveOutputVolts = driveSimState.getMotorVoltage();
    inputs.driveStatorCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void changeDriveSetpoint(double metersPerSecond, boolean openLoop) {
    drive.setControl(
        openLoop
            ? driveOpenLoop.withOutput(
                RobotController.getBatteryVoltage()
                    * (metersPerSecond * DrivebaseConstants.kDriveRotorToMeters)
                    / (Units.radiansPerSecondToRotationsPerMinute(
                            DrivebaseConstants.kDriveMotor.freeSpeedRadPerSec)
                        / 60d))
            : drivePID.withVelocity(metersPerSecond));
  }

  @Override
  public void changeTurnSetpoint(Rotation2d rotation) {
    turnSim.setInputVoltage(
        MathUtil.clamp(
            turnPID.calculate(turnSim.getAngularPositionRotations(), rotation.getRotations()),
            -12.0,
            12.0));
  }
}
