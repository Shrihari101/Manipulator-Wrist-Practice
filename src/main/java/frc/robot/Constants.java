package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class Constants {

  public class ManipulatorRollerConstants {
    public static final double kGearing = 1.5;
    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kRollerSimMOI = 1;
    public static final double kRollerSimGearing = 10;
    public static final double kRollerPositionTolerance = 2;

    public static final LoggedTunableNumber rollerkP = new LoggedTunableNumber("P", 0.0);
    public static final LoggedTunableNumber rollerkI = new LoggedTunableNumber("I", 0.0);
    public static final LoggedTunableNumber rollerkD = new LoggedTunableNumber("D", 0.0);
    public static final LoggedTunableNumber rollerkS = new LoggedTunableNumber("kS", 0.0);

    public static final LoggedTunableNumber rollerkSimP = new LoggedTunableNumber("SimP", 0.0);
    public static final LoggedTunableNumber rollerkSimI = new LoggedTunableNumber("SimI", 0.0);
    public static final LoggedTunableNumber rollerkSimD = new LoggedTunableNumber("SimD", 0.0);
    public static final LoggedTunableNumber rollerkSimkS = new LoggedTunableNumber("SimkS", 0.0);
    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Manipulator Roller Stow Voltage", 4.0);
    public static final LoggedTunableNumber kRollerIntakingVoltage =
        new LoggedTunableNumber("Manipulator Roller Intaking Voltage", 10.0);
  }

  public class ManipulatorWristConstants {
    public static final double minAngle = 0;
    public static final double maxAngle = 135;

    public static final double kWristAbsoluteEncoderGearRatio = 32.0 / 14.0;
    public static final Rotation2d kWristOffset =
        Rotation2d.fromDegrees((-37.4) * kWristAbsoluteEncoderGearRatio);

    public static final LoggedTunableNumber wristkP = new LoggedTunableNumber("P", 0.0);
    public static final LoggedTunableNumber wristkI = new LoggedTunableNumber("I", 0.0);
    public static final LoggedTunableNumber wristkD = new LoggedTunableNumber("D", 0.0);
    public static final LoggedTunableNumber wristkS = new LoggedTunableNumber("kS", 0.0);

    public static final LoggedTunableNumber wristkSimP = new LoggedTunableNumber("SimP", 0.0);
    public static final LoggedTunableNumber wristkSimI = new LoggedTunableNumber("SimI", 0.0);
    public static final LoggedTunableNumber wristkSimD = new LoggedTunableNumber("SimD", 0.0);
    public static final LoggedTunableNumber wristkSimkS = new LoggedTunableNumber("SimkS", 0.0);
    public static final LoggedTunableNumber kTuningAngle =
        new LoggedTunableNumber("Wrist Tuning Angle", 25);

    public static final double kWristTolerance = 2.0;

    public static final Rotation2d kStowAngle = Rotation2d.fromDegrees(90);
    public static final Rotation2d kIntakingAngle = Rotation2d.fromDegrees(45);

    public static final double kLengthMeters = Units.inchesToMeters(10);
    public static final double kMassKg = Units.lbsToKilograms(2);
    public static final double kGearing = (66.0 / 10.0) * (32.0 / 14.0);
    public static final DCMotor kWristSimGearbox = DCMotor.getKrakenX60(1);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(135);
    public static final Rotation2d kStartingAngle = Rotation2d.fromDegrees(28);
  }

  public class Ports {
    public static final String kMainCanivoreName = "Main";
    public static final int kManipulatorWrist = 30;
    public static final int kManipulatorAbsoluteEncoder = 1;
    public static final int kManipulatorRoller = 2;
  }

  public class ProtoConstants {
    public static final boolean kRealManipulator = false;
  }

  public class CurrentLimitConstants {
    public static final double kManipulatorWristDefaultStatorLimit = 120.0;
    public static final double kManipulatorWristDefaultSupplyLimit = 80.0;
    public static final double kManipulatorRollerDefaultStatorLimit = 10.0;
    public static final double kManipulatorRollerDefaultSupplyLimit = 20.0;
  }

  public static final boolean kTuningMode = true;
  public static final boolean kUseBaseRefreshManager = false;
}
