package frc.robot;

import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.manipulator.Manipulator;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;

public class RobotState {
  private Indexer m_indexer;
  private Manipulator m_manipulator;
  private Elevator m_elevator;

  public enum RobotAction {
    kStow,
    kTuning,
    kStop,
    kIndexing,
    kIntaking,
    // add more elevator states here when done
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private RobotState(Indexer indexer, Manipulator manipulator, Elevator elevator) {
    m_indexer = indexer;
    m_manipulator = manipulator;
    m_elevator = elevator;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kStow, this::stowPeriodic);
    periodicHash.put(RobotAction.kIndexing, this::indexingPeriodic);
    periodicHash.put(RobotAction.kIntaking, this::intakingPeriodic);
    periodicHash.put(RobotAction.kStop, this::stopPeriodic);
    periodicHash.put(RobotAction.kTuning, this::tuningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTuning);
  }

  public boolean getWristAtSetpoint() {
    return m_manipulator.atSetpoint();
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
  }

  public void stowPeriodic() {
    m_manipulator.stowPeriodic();
  }

  public void intakingPeriodic() {
    m_manipulator.intakingPeriodic();
  }

  public void indexingPeriodic() {
    m_indexer.indexingPeriodic();
  }

  public void stopPeriodic() {
    m_indexer.stopPeriodic();
  }

  public void tuningPeriodic() {
    m_indexer.tuningPeriodic();
    m_manipulator.tuningPeriodic();
  }
}
