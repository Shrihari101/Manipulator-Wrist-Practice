package frc.robot.Subsystems.manipulator.roller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorRollerConstants;

public class RollerIOSim implements RollerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;
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

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    rollerInputs.velocity = m_sim.getAngularVelocityRPM() / 60.0;
    rollerInputs.acceleration = getAcceleration();
    rollerInputs.current = m_sim.getCurrentDrawAmps();
    rollerInputs.voltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public void setCurrentLimits(double currentLimits) {}

  private double getAcceleration() {
    return Units.radiansToRotations(m_sim.getAngularAccelerationRadPerSecSq());
  }
}
