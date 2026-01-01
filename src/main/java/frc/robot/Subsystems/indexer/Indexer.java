package frc.robot.Subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;

  public final IndexerInputsAutoLogged m_inputs = new IndexerInputsAutoLogged();

  public static enum IndexerState {
    kIndexing,
    kTuning,
    kStop,
  }

  private SubsystemProfiles<IndexerState> m_profiles;

  public Indexer(IndexerIO io) {
    m_io = io;

    Map<IndexerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IndexerState.kIndexing, this::indexingPeriodic);
    periodicHash.put(IndexerState.kTuning, this::tuningPeriodic);
    periodicHash.put(IndexerState.kStop, this::stopPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, IndexerState.kStop);
  }

  @Override
  public void periodic() {

    if (IndexerConstants.kFullTuningMode) {
      updateState(IndexerState.kTuning);
    }

    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.recordOutput("Indexer/State", m_profiles.getCurrentProfile());
    Logger.processInputs("Indexer", m_inputs);
  }

  public void updateState(IndexerState state) {
    if (m_profiles.getCurrentProfile() == IndexerState.kTuning) {
      return;
    }

    m_profiles.setCurrentProfile(state);
  }

  public IndexerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void stopPeriodic() {
    m_io.setVoltage(IndexerConstants.kStopVoltage.get(), IndexerConstants.kStopVoltage.get());
  }

  public void indexingPeriodic() {
    m_io.setVoltage(
        IndexerConstants.kIndexingVoltage.get(), IndexerConstants.kTopIndexingVoltage.get());
  }

  public void tuningPeriodic() {
    m_io.setVoltage(IndexerConstants.kTuningVoltage.get(), IndexerConstants.kTuningVoltage.get());
  }
}
