package frc.robot.Subsystems.roller;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorRollerConstants;

public class RollerIOSim implements RollerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;
  private boolean m_positionControl = false;
  private PIDController m_PidController = new PIDController(0.0, 0.0, 0.0);

  public RollerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            ManipulatorRollerConstants.kRollerSimGearbox,
            ManipulatorRollerConstants.kRollerSimMOI,
            ManipulatorRollerConstants.kRollerSimGearing);

    m_sim = new DCMotorSim(plant, ManipulatorRollerConstants.kRollerSimGearbox);

    m_PidController.setTolerance(ManipulatorRollerConstants.kRollerPositionTolerance);
  }

  @Override
  public void updateInputs(RollerInputsAutoLogged rollerInputs) {
    if (m_positionControl) {
      m_voltage = m_PidController.calculate(getPosition().in(Degrees));
    }

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    rollerInputs.currentAngle = getPosition().in(Degrees);
    rollerInputs.positionControl = m_positionControl;
    rollerInputs.desiredAngle = m_PidController.getSetpoint();
    rollerInputs.velocity = m_sim.getAngularVelocityRPM() / 60.0;
    rollerInputs.acceleration = getAcceleration();
    rollerInputs.current = m_sim.getCurrentDrawAmps();
    rollerInputs.voltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_positionControl = false;
  }

  @Override
  public void setDesiredAngle(Angle position) {
    m_positionControl = true;
    m_PidController.setSetpoint(position.in(Degrees));
  }

  private Angle getPosition() {
    return Radians.of(m_sim.getAngularPositionRad());
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS) {
    m_PidController.setPID(kP, kI, kD);
  }

  @Override
  public void setCurrentLimits(double currentLimits) {
    // TODO: Ask shorya what to put here
  }

  private double getAcceleration() {
    return Units.radiansToRotations(m_sim.getAngularAccelerationRadPerSecSq());
  }
}
