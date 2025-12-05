package frc.robot.Subsystems.manipulator.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorWristConstants;
import frc.robot.Constants.Ports;

public class WristIOKraken implements WristIO {
    
            private TalonFX m_motor;
            private DutyCycleEncoder m_absoluteEncoder;
            private final TalonFXConfiguration m_config;
        
            private StatusSignal<Angle> m_motorAngle;
            private StatusSignal<Current> m_motorCurrent;
            private StatusSignal<AngularVelocity> m_motorVelocity;
            private StatusSignal<Current> m_motorStatorCurrent;
            private StatusSignal<Voltage> m_motorVoltage;
            private StatusSignal<Temperature> m_motorTemperature;
            private StatusSignal<ConnectedMotorValue> m_connected;
        
            private boolean m_relativeEncoderReset = false;
            private PositionVoltage m_positionControl = new PositionVoltage(0.0).withSlot(0);
            private Rotation2d m_desiredAngle = new Rotation2d();
        
            public WristIOKraken(int port, int absoluteEncoderPort) {
                m_motor = new TalonFX(port, Ports.kMainCarnivoreName); 
                m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);
        
                var currentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultSupplyLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultStatorLimit);
        
            var motorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive);
        
            var feedback =
                new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorWristConstants.kWristGearRatio);
        
            m_config =
                new TalonFXConfiguration()
                    .withCurrentLimits(currentLimits)
                    .withMotorOutput(motorOutput)
                    .withFeedback(feedback);
        
            m_motor.getConfigurator().apply(m_config); //kent's configs
        
                m_connected = m_motor.getConnectedMotor();
                m_motorAngle = m_motor.getPosition();
                m_motorCurrent = m_motor.getSupplyCurrent();
                m_motorVelocity = m_motor.getVelocity();
                m_motorStatorCurrent = m_motor.getStatorCurrent();
                m_motorVoltage = m_motor.getMotorVoltage();
                m_motorTemperature = m_motor.getDeviceTemp();
        
                StatusSignal.setUpdateFrequencyForAll(
                    100,
                    m_motorVoltage,
                    m_motorVelocity,
                    m_connected,
                    m_motorStatorCurrent,
                    m_motorCurrent,
                    m_motorTemperature);
                }
        
            @Override
            public void updateInputs(WristInputs inputs) {
                if(!m_relativeEncoderReset && m_absoluteEncoder.get() != 1) {
                    m_relativeEncoderReset = true;
                    resetRelativeEncoder();
                }
        
            inputs.currentAngleDeg = Rotation2d.fromRotations(m_motorAngle.getValueAsDouble()).getDegrees();
            inputs.desiredAngleDeg = m_desiredAngle.getDegrees();
            inputs.atSetpoint = atSetpoint();
            inputs.current = m_motorCurrent.getValueAsDouble();
            inputs.voltage = m_motorVoltage.getValueAsDouble();
            inputs.temperature = m_motorTemperature.getValueAsDouble();
            inputs.velocityRPS = m_motorVelocity.getValueAsDouble();
            inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
            inputs.motorIsConnected = m_connected.getValue() != ConnectedMotorValue.Unknown;
        
            }
        
            @Override
            public void setPIDFF(int slot, double p, double i, double d, double kS) {
               var config = new SlotConfigs().withKP(p).withKI(i).withKD(d).withKS(kS);
               config.SlotNumber = 0;
               m_motor.getConfigurator().apply(config);
            }
        
            @Override
            public void setDesiredAngle(Rotation2d angle) {
                double angleAmount = angle.getDegrees();
                angleAmount = MathUtil.clamp(angleAmount, ManipulatorWristConstants.minAngleAmount, ManipulatorWristConstants.maxAngleAmount);
        angle = Rotation2d.fromDegrees(angleAmount);
        m_desiredAngle = angle;
        m_motor.setControl(m_positionControl.withPosition(angle.getRotations()));
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(m_motorAngle.getValueAsDouble());
    }

    private boolean atSetpoint() {
        return Math.abs(m_desiredAngle.getDegrees() - getCurrentAngle().getDegrees()) < 5.0;
    }

    public void resetRelativeEncoder() {
        double raw = m_absoluteEncoder.get();
        raw += ManipulatorWristConstants.kWristOffset.getRotations();
        raw %= 1;
        Rotation2d rotations = 
            Rotation2d.fromRotations(raw).div(ManipulatorWristConstants.kWristAbsoluteEncoderGearRatio);
            Rotation2d.fromRotations(raw).plus(Rotation2d.fromDegrees(23));
            Rotation2d.fromRotations(raw).getRotations();
        m_motor.setPosition(rotations.getRotations());
    }
}
