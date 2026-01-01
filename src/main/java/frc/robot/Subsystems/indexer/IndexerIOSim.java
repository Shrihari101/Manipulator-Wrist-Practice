package frc.robot.Subsystems.indexer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {

  private DCMotorSim m_sideSim;
  private DCMotorSim m_topSim;

  private double m_sideVoltage = 6.0;
  private double m_topVoltage = 7.0;

  public IndexerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimMOI, IndexerConstants.kSimGearing);
    m_sideSim = new DCMotorSim(plant, IndexerConstants.kSimGearbox);
    m_topSim = new DCMotorSim(plant, IndexerConstants.kSimGearbox);
  }

  @Override
  public void setVoltage(double sideVoltage, double topVoltage) {
    m_sideVoltage = sideVoltage;
    m_topVoltage = topVoltage;
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    m_sideSim.setInputVoltage(m_sideVoltage);
    m_sideSim.update(0.02);
    m_topSim.setInputVoltage(m_topVoltage);
    m_topSim.update(0.02);

    inputs.sidePosition = m_sideSim.getAngularPositionRotations();
    inputs.sideVelocity = m_sideSim.getAngularVelocityRPM();
    inputs.sideCurrent = m_sideSim.getCurrentDrawAmps();
    inputs.sideVoltage = m_sideVoltage;
    inputs.topPosition = m_topSim.getAngularPositionRotations();
    inputs.topVelocity = m_topSim.getAngularVelocityRPM();
    inputs.topCurrent = m_topSim.getCurrentDrawAmps();
    inputs.topVoltage = m_topVoltage;
  }

  @Override
  public void setCurrentLimits(double currentLimits) {}
}
