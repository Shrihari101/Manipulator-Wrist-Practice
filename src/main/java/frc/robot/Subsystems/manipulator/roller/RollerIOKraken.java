package frc.robot.Subsystems.manipulator.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorRollerConstants;
import frc.robot.Constants.Ports;

public class RollerIOKraken implements RollerIO {

  private TalonFX m_motor;
  private final TalonFXConfiguration m_config;

  private StatusSignal<Angle> m_motorAngle;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;
  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<AngularAcceleration> m_motorAcceleration;

  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public RollerIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorRollerConstants.kGearing);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config); // kent's configs
    m_motor.getConfigurator().setPosition(0.0);

    m_connected = m_motor.getConnectedMotor();
    m_motorAngle = m_motor.getPosition();
    m_motorVelocity = m_motor.getVelocity();
    m_motorAcceleration = m_motor.getAcceleration();
    m_motorTemperature = m_motor.getDeviceTemp();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, m_motorAngle, m_motorStatorCurrent, m_motorCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connected,
        m_motorAcceleration,
        m_motorVelocity,
        m_motorVoltage,
        m_motorTemperature);
  }

  @Override
  public void updateInputs(RollerInputsAutoLogged rollerInputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_connected,
          m_motorAngle,
          m_motorVelocity,
          m_motorAcceleration,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorVoltage,
          m_motorTemperature);
    }
    rollerInputs.motorIsConnected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    rollerInputs.velocity = m_motorVelocity.getValue().in(RotationsPerSecond);
    rollerInputs.acceleration = m_motorAcceleration.getValue().in(RotationsPerSecondPerSecond);
    rollerInputs.temperature = m_motorTemperature.getValue().in(Celsius);
    rollerInputs.voltage = m_motorVoltage.getValue().in(Volts);
    rollerInputs.current = m_motorCurrent.getValue().in(Amps);
    rollerInputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setControl(m_voltageOut.withOutput(voltage));
  }

  @Override
  public void setCurrentLimits(double currentLimits) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(currentLimits), 0.0);
  }
}
