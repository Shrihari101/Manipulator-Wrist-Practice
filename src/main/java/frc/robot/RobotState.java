package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReefHeight;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.Elevator.ElevatorState;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.indexer.Indexer.IndexerState;
import frc.robot.Subsystems.manipulator.Manipulator;
import frc.robot.Subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private Indexer m_indexer;
  private Manipulator m_manipulator;
  private Elevator m_elevator;
  private double m_desiredHeight;
  private ReefHeight m_desiredReefHeight = ReefHeight.L1;

  public enum RobotAction {
    kIntaking,
    kDefault,
    kScoring,
  }

  private static RobotState m_instance;
  private SubsystemProfiles<RobotAction> m_profiles;

  private RobotState(Indexer indexer, Manipulator manipulator, Elevator elevator) {
    m_indexer = indexer;
    m_manipulator = manipulator;
    m_elevator = elevator;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kDefault, () -> {});
    periodicHash.put(RobotAction.kIntaking, this::intakingPeriodic);
    periodicHash.put(RobotAction.kScoring, this::scoringPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kDefault);
  }

  public static RobotState startInstance(
      Indexer indexer, Manipulator manipulator, Elevator elevator) {
    if (m_instance == null) {
      m_instance = new RobotState(indexer, manipulator, elevator);
    }
    return m_instance;
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public boolean getWristAtSetpoint() {
    return m_manipulator.atSetpoint();
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotState/CurrentAction", m_profiles.getCurrentProfile());
  }

  public void intakingPeriodic() {
    if (m_manipulator.atSetpoint() && m_elevator.atSetpoint()) {
      m_indexer.updateState(IndexerState.kIndexing);
    } else {
      m_indexer.updateState(IndexerState.kStop);
    }
  }

  public void scoringPeriodic() {
    m_elevator.setDesiredHeight(ElevatorConstants.kL4);
  }

  public void setDesiredHeight(double desiredHeight) {
    m_desiredHeight = desiredHeight;
  }

  public void setDesiredReefHeight(ReefHeight height) {
    m_desiredReefHeight = height;
  }

  public ReefHeight getDesiredReefHeight() {
    return m_desiredReefHeight;
  }

  public void setDefaultAction() {
    updateRobotAction(RobotAction.kDefault);
  }

  public void updateRobotAction(RobotAction newAction) {
    IndexerState newIndexerState = IndexerState.kStop;
    ElevatorState newElevatorState = ElevatorState.kStow;
    ManipulatorState newManipulatorState = ManipulatorState.kStow;

    switch (newAction) {
      case kDefault:
        break;

      case kIntaking:
        newElevatorState = ElevatorState.kIntaking;
        newManipulatorState = ManipulatorState.kIntaking;
        break;

      case kScoring:
        newIndexerState = m_indexer.getCurrentState();
        newElevatorState = m_elevator.getCurrentState();
        newManipulatorState = m_manipulator.getCurrentState();

        m_manipulator.runRollerScoring();
        break;
    }

    m_profiles.setCurrentProfile(newAction);

    if (m_indexer.getCurrentState() == newIndexerState) {
      m_indexer.updateState(newIndexerState);
    }

    if (m_elevator.getCurrentState() != newElevatorState) {
      m_elevator.updateState(newElevatorState);
    }

    if (m_manipulator.getCurrentState() != newManipulatorState) {
      m_manipulator.updateState(newManipulatorState);
    }
  }

  public RobotAction getCurrentAction() {
    return m_profiles.getCurrentProfile();
  }

  public IndexerState getIndexerState() {
    return m_indexer.getCurrentState();
  }

  public ManipulatorState getManipulatorState() {
    return m_manipulator.getCurrentState();
  }

  public ElevatorState getElevatorState() {
    return m_elevator.getCurrentState();
  }
}
