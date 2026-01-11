package frc.robot.Subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim m_sim;
  private PIDController m_controller;
  private double m_voltage;
  private boolean m_positionControl = true;

  public ElevatorIOSim() {
    var plant =
        LinearSystemId.createElevatorSystem(
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kSimMOI,
            ElevatorConstants.kRadius,
            ElevatorConstants.kSimGearing);
    m_sim =
        new ElevatorSim(
            plant,
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kMinHeight,
            ElevatorConstants.kMaxHeight,
            true,
            ElevatorConstants.kMinHeight);

    m_controller = new PIDController(0, 0, 0);
    m_controller.setTolerance(ElevatorConstants.kHeightTolerance);
  }

  public void updateInputs(ElevatorInputs inputs) {
    if (m_positionControl) {
      m_voltage = m_controller.calculate(Units.metersToInches(m_sim.getPositionMeters()));
    }

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.leadingPosition = Units.metersToInches(m_sim.getPositionMeters());
    inputs.followingPosition = Units.metersToInches(m_sim.getPositionMeters());
    inputs.leadingVelocity = m_sim.getVelocityMetersPerSecond();
    inputs.leadingVoltage = m_voltage;
    inputs.leadingSupplyCurrent = m_sim.getCurrentDrawAmps();
    inputs.positionControl = m_positionControl;
    inputs.desiredLocation = m_controller.getSetpoint();
    inputs.atSetpoint = atSetpoint();
  }

  public void setDesiredHeight(double inches) {
    m_controller.setSetpoint(inches);
    m_positionControl = true;
  }

  public void setPIDFF(int slot, double kP, double kI, double kD, double kG) {
    m_controller.setPID(kP, kI, kD);
  }

  public void setCurrentLimits(double currentLimits) {}

  private boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_positionControl = false;
  }
}
