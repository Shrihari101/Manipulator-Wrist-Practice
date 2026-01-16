package frc.robot.Subsystems.elevator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReefHeight;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_io;
  public final ElevatorInputsAutoLogged m_inputs;
  private ReefHeight m_desiredReefHeight;
  private double m_desiredHeight;

  public static enum ElevatorState {
    kStow,
    kIntaking,
    kScoring,
    kSlamming,
  }

  private SubsystemProfiles<ElevatorState> m_profiles;

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();

    Map<ElevatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ElevatorState.kStow, this::stowPeriodic);
    periodicHash.put(ElevatorState.kScoring, this::scoringPeriodic);
    periodicHash.put(ElevatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ElevatorState.kSlamming, this::slammingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, ElevatorState.kStow);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_io.setPIDFF(
                0,
                ElevatorConstants.kP.get(),
                ElevatorConstants.kI.get(),
                ElevatorConstants.kD.get(),
                ElevatorConstants.kS.get());
          } else {
            m_io.setPIDFF(
                0,
                ElevatorConstants.kSimP.get(),
                ElevatorConstants.kSimI.get(),
                ElevatorConstants.kSimD.get(),
                ElevatorConstants.kSimkS.get());
          }
        },
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kS,
        ElevatorConstants.kSimP,
        ElevatorConstants.kSimI,
        ElevatorConstants.kSimD,
        ElevatorConstants.kSimkS);

    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();
    m_io.setDesiredHeight(m_inputs.desiredLocation);

    Logger.processInputs("Elevator", m_inputs);
    Logger.recordOutput("Elevator/State", m_profiles.getCurrentProfile());
  }

  public void updateState(ElevatorState state) {
    m_profiles.setCurrentProfile(state);
  }

  public ElevatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredReefHeight(double m_desiredReefHeight) {
    if (m_profiles.getCurrentProfile() == ElevatorState.kScoring) {
      m_io.setDesiredHeight(m_desiredHeight);
    }
  }

  public double getCurrentHeight() {
    return m_inputs.leadingPosition;
  }

  public ReefHeight getDesiredReefHeight() {
    return m_desiredReefHeight;
  }

  public boolean atSetpoint() {
    return Math.abs(m_inputs.leadingPosition - m_inputs.desiredLocation)
        <= ElevatorConstants.kHeightTolerance;
  }

  public void slammingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kSlammingHeight.get());
  }

  public void intakingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight.get());
  }

  public void stowPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kStowHeight.get());
  }

  public void scoringPeriodic() {
    m_io.setDesiredHeight(m_desiredHeight);
  }

  public void setDesiredHeight(double desiredHeight) {
    m_desiredHeight = desiredHeight;
  }
}
