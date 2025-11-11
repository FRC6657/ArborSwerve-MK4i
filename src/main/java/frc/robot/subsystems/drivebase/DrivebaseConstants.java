package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DrivebaseConstants {

  public static int kOdometryFrequency = 150; // Hz

  // Drivebase CAN IDs
  public static enum CAN {
    FR_D(1),
    FL_D(2),
    BR_D(3),
    BL_D(4),
    FR_T(5),
    FL_T(6),
    BR_T(7),
    BL_T(8),
    FR_E(9),
    FL_E(10),
    BR_E(11),
    BL_E(12),
    Gyro(13);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  // Physical Characteristics
  public static final double kFrameWidth = Units.inchesToMeters(29); // +- Y Direction
  public static final double kFrameLength = Units.inchesToMeters(29); // +- X Direction
  public static final double kMK4iTW_Offset = Units.inchesToMeters(2.625);

  public static final double kTrackWidthX = Units.inchesToMeters(kFrameLength - kMK4iTW_Offset * 2);
  public static final double kTrackWidthY = Units.inchesToMeters(kFrameWidth - kMK4iTW_Offset * 2);

  public static final Translation2d[] kModuleTranslations = {
    new Translation2d(kTrackWidthX / 2, kTrackWidthY / 2),
    new Translation2d(kTrackWidthX / 2, -kTrackWidthY / 2),
    new Translation2d(-kTrackWidthX / 2, kTrackWidthY / 2),
    new Translation2d(-kTrackWidthX / 2, -kTrackWidthY / 2)
  };

  public record ModuleConstants(String name, int driveID, int turnID, int encoderID) {}

  public static final ModuleConstants kFrontLeftModuleConstants =
      new ModuleConstants("Front Left", CAN.FL_D.id, CAN.FL_T.id, CAN.FL_E.id);
  public static final ModuleConstants kFrontRightModuleConstants =
      new ModuleConstants("Front Right", CAN.FR_D.id, CAN.FR_T.id, CAN.FR_E.id);
  public static final ModuleConstants kBackLeftModuleConstants =
      new ModuleConstants("Back Left", CAN.BL_D.id, CAN.BL_T.id, CAN.BL_E.id);
  public static final ModuleConstants kBackRightModuleConstants =
      new ModuleConstants("Back Right", CAN.BR_D.id, CAN.BR_T.id, CAN.BR_E.id);

  // Enum representation of the MK4i Ratio Options
  public static enum MK4i_Ratio {
    L1(19d / 25d),
    L2(17d / 27d),
    L3(16d / 28d);

    public double ratio;

    MK4i_Ratio(double ratio) {
      this.ratio = ratio;
    }
  }

  public static final DCMotor kDriveMotor = DCMotor.getFalcon500(1);
  public static final DCMotor kTurnMotor = DCMotor.getFalcon500(1);
  public static final double kDriveWheelDiameter = Units.inchesToMeters(4);

  public static final int kDrivePinionTeeth = 14;
  public static final double kDriveRatio =
      (45d / 15d) * MK4i_Ratio.L3.ratio * (50d / kDrivePinionTeeth);
  public static final double kTurnRatio = (150d / 7d);
  public static final double kDriveRotorToMeters = kDriveRatio / (kDriveWheelDiameter * Math.PI);

  public static final double kMaxLinearSpeed =
      (Units.radiansPerSecondToRotationsPerMinute(kDriveMotor.freeSpeedRadPerSec) / 60d)
          * 1d
          / kDriveRatio
          * (kDriveWheelDiameter * Math.PI); // Meters Per Second
  public static final double kMaxLinearAcceleration =
      8; // Meters per second per second (rough underestimate of the traction limit)
  public static final double kMaxAngularSpeed =
      kMaxLinearSpeed / Math.hypot(kTrackWidthX / 2d, kTrackWidthY / 2d); // Radians per second

  // Motor Configurations
  public static TalonFXConfiguration driveConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kDriveRotorToMeters))
          .withSlot0(new Slot0Configs().withKV(12d / kMaxLinearSpeed).withKS(0).withKP(2.25));

  public static TalonFXConfiguration turnConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(20)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kTurnRatio))
          .withSlot0(new Slot0Configs().withKS(0.27).withKP(25).withKD(0.7))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));


  


  // Choreo PIDControllers

  public static final PIDController kChoreoXController = new PIDController(1.6, 0, 0);
  public static final PIDController kChoreoYController = new PIDController(1.6, 0, 0);
  public static final PIDController kChoreoThetaController = new PIDController(1.2, 0, 0);
}
