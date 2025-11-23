package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robot.util.LoggedTunableNumber;

public class Constants {
    public class ManipulatorConstants {
        public static final double kWristGearRatio = 0;
        public static final double something = 1;
        public static final int minAngleAmount = 0;
        public static final int maxAngleAmount = 140;

        public static final double kWristAbsoluteEncoderGearRatio = 32.0 / 14.0;
        public static final Rotation2d kWristOffset = 
            Rotation2d.fromDegrees((-37.4)* kWristAbsoluteEncoderGearRatio);

        public static final LoggedTunableNumber kP = new LoggedTunableNumber("P");
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("I");
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("D");
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("kS");

        public static final LoggedTunableNumber kSimP = new LoggedTunableNumber("P");
        public static final LoggedTunableNumber kSimI = new LoggedTunableNumber("I");
        public static final LoggedTunableNumber kSimD = new LoggedTunableNumber("D");
        public static final LoggedTunableNumber kSimkS = new LoggedTunableNumber("kS");
        public static final int kWristTolerance = 0;

    }
    public class Ports {
        public static final String kMainCarnivoreName = "Main";
    }

    public class CurrentLimitConstants {
        public static final double kManipulatorWristDefaultStatorLimit = 120.0;
        public static final double kManipulatorWristDefaultSupplyLimit = 80.0;

    }

    public static final boolean kTuningMode = false;//for loggedTunableNumber file to work
}
