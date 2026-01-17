package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class Constants {

  public class ElevatorConstants {
    public static final double kL1 = 9.5;
    public static final double kL2 = 27.0;
    public static final double kL3 = 43.25;
    public static final double kL4 = 71.0;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("ElevatorP", 0.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("ElevatorI", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("ElevatorD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("ElevatorkS", 0.0);

    public static final LoggedTunableNumber kSimP = new LoggedTunableNumber("ElevatorSimP", 0.01);
    public static final LoggedTunableNumber kSimI = new LoggedTunableNumber("ElevatorSimI", 0.0);
    public static final LoggedTunableNumber kSimD = new LoggedTunableNumber("ElevatorSimD", 0.0);
    public static final LoggedTunableNumber kSimkS = new LoggedTunableNumber("ElevatorSimkS", 0.0);
    public static final double kHeightTolerance = 0.5;
    public static final double kMinHeight = 0.0;
    public static final double kMaxHeight = 73.5;
    public static final double kElevatorDefaultSupplyLimit = 10;
    public static final double kElevatorDefaultStatorLimit = 8;
    public static final LoggedTunableNumber kSlammingHeight =
        new LoggedTunableNumber("Slamming Elevator Setpoint", 0.0);
    public static final LoggedTunableNumber kIntakingHeight =
        new LoggedTunableNumber("Intaking Elevator Setpoint", 0.0);
    public static final LoggedTunableNumber kStowHeight =
        new LoggedTunableNumber("Stow Elevator Setpoint", 2.0);
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kRadius = 2;
    public static final double kSimMOI = 0.001;
    public static final double kGearRatio = 54.0 / 12.0;
    public static final double kSimGearing = kGearRatio;
    public static final double kSensorToMechanismRatio = (kGearRatio / (kRadius * Math.PI));
  }

  public class IndexerConstants {
    public static final LoggedTunableNumber kStopVoltage =
        new LoggedTunableNumber("Indexer Stop Voltage", 0.0);
    public static final LoggedTunableNumber kIndexingVoltage =
        new LoggedTunableNumber("Indexer Indexing Voltage", 2.0);
    public static final LoggedTunableNumber kTopIndexingVoltage =
        new LoggedTunableNumber("Indexer Top Indexing Voltage", 3.0);
    public static final LoggedTunableNumber kTuningVoltage =
        new LoggedTunableNumber("Indexer Tuning Voltage", 1.0);
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kSimMOI = 1;
    public static final double kSimGearing = 10;
    public static final double kGearRatio = 0;
    public static final boolean kFullTuningMode = false;
  }

  public class ManipulatorRollerConstants {
    public static final LoggedTunableNumber kRollerL1ScoringVoltageManual =
        new LoggedTunableNumber("Manipulator Roller L1 Scoring Voltage Manual", 2.25);

    public static final LoggedTunableNumber kRollerL2L3ScoringVoltage =
        new LoggedTunableNumber("Manipulator Roller L2 L3 Scoring Voltage", 3.0);
    public static final LoggedTunableNumber kRollerL4ScoringVoltage =
        new LoggedTunableNumber("Manipulator Roller L4 Scoring Voltage", 3.0);

    public static final double kGearing = 1.5;
    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kRollerSimMOI = 1;
    public static final double kRollerSimGearing = 10;
    public static final double kRollerPositionTolerance = 2;
    public static final boolean kTuningMode = false;
    public static final LoggedTunableNumber kTuningVoltage =
        new LoggedTunableNumber("Roller Tuning Voltage", 0.0);
    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Manipulator Roller Stow Voltage", 4.0);
    public static final LoggedTunableNumber kRollerIntakingVoltage =
        new LoggedTunableNumber("Manipulator Roller Intaking Voltage", 10.0);
  }

  public class ManipulatorWristConstants {
    public static final double kMinAngleDeg = 0;
    public static final double kMaxAngleDeg = 135;

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
    public static final String kDriveCanivoreName = "Indexer";
    public static final int kIndexerSideMotor = 3;
    public static final int kIndexerTopMotor = 4;
    public static final int kElevatorLead = 5;
    public static final int kElevatorFollowing = 6;
  }

  public class ProtoConstants {
    public static final boolean kRealManipulator = false;
  }

  public class CurrentLimitConstants {
    public static final double kManipulatorWristDefaultStatorLimit = 120.0;
    public static final double kManipulatorWristDefaultSupplyLimit = 80.0;
    public static final double kManipulatorRollerDefaultStatorLimit = 10.0;
    public static final double kManipulatorRollerDefaultSupplyLimit = 20.0;
    public static final double kIndexerDefaultSupplyLimit = 20.0;
    public static final double kIndexerDefaultStatorLimit = 10.0;
  }

  public static final boolean kTuningMode = true;
  public static final boolean kUseBaseRefreshManager = false;
}
