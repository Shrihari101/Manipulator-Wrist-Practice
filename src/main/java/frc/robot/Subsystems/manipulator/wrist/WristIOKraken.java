package frc.robot.Subsystems.manipulator.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorWristConstants;
import frc.robot.Constants.Ports;

public class WristIOKraken implements WristIO {

  private TalonFX m_motor;
  // private DutyCycleEncoder m_absoluteEncoder;
  private final TalonFXConfiguration m_config;

  private StatusSignal<Angle> m_motorAngle;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;
  private StatusSignal<ConnectedMotorValue> m_connected;

  // private boolean m_relativeEncoderReset = false;
  private PositionVoltage m_positionControl = new PositionVoltage(0.0).withSlot(0);
  private Rotation2d m_desiredAngle = new Rotation2d();

  public WristIOKraken(int port, int absoluteEncoderPort) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);
    // m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);

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
        new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorWristConstants.kGearing);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotorOutput(motorOutput)
            .withFeedback(feedback);

    m_motor.getConfigurator().apply(m_config); // kent's configs

    m_motor.setPosition(Degrees.of(100));

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
        m_motorTemperature,
        m_motorAngle);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    // if (!m_relativeEncoderReset && m_absoluteEncoder.get() != 1) {
    //   m_relativeEncoderReset = true;
    //   resetRelativeEncoder();
    // }
    BaseStatusSignal.refreshAll(
        m_motorVoltage,
        m_motorVelocity,
        m_connected,
        m_motorStatorCurrent,
        m_motorCurrent,
        m_motorTemperature,
        m_motorAngle);

    inputs.currentAngleDeg = m_motorAngle.getValue().in(Degrees);
    inputs.desiredAngleDeg = m_desiredAngle.getDegrees();
    inputs.atSetpoint = atSetpoint();

    inputs.current = m_motorCurrent.getValue().in(Amps);
    inputs.voltage = m_motorVoltage.getValue().in(Volts);
    inputs.motorTemperature = m_motorTemperature.getValue().in(Celsius);
    inputs.velocityRPS = m_motorVelocity.getValue().in(RotationsPerSecond);
    inputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
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
    angleAmount =
        MathUtil.clamp(
            angleAmount,
            ManipulatorWristConstants.kMinAngleDeg,
            ManipulatorWristConstants.kMaxAngleDeg);
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

  // public void resetRelativeEncoder() {
  //   double raw = m_absoluteEncoder.get();
  //   raw += ManipulatorWristConstants.kWristOffset.getRotations();
  //   raw %= 1;
  //   Rotation2d rotations =
  //
  // Rotation2d.fromRotations(raw).div(ManipulatorWristConstants.kWristAbsoluteEncoderGearRatio);
  //   Rotation2d.fromRotations(raw).plus(Rotation2d.fromDegrees(23));
  //   Rotation2d.fromRotations(raw).getRotations();
  //   m_motor.setPosition(rotations.getRotations());
  // }
}
