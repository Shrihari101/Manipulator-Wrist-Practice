package frc.robot.Subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorRollerConstants;
import frc.robot.Constants.ManipulatorWristConstants;
import frc.robot.Constants.ReefHeight;
import frc.robot.RobotState;
import frc.robot.Subsystems.manipulator.roller.RollerIO;
import frc.robot.Subsystems.manipulator.roller.RollerInputsAutoLogged;
import frc.robot.Subsystems.manipulator.wrist.WristIO;
import frc.robot.Subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private WristIO m_wristIO;
  private RollerIO m_rollerIO;
  private Rotation2d m_desiredAngle = new Rotation2d();
  private boolean m_runRollerScoring = false;

  public final WristInputsAutoLogged m_wristInputs = new WristInputsAutoLogged();
  public final RollerInputsAutoLogged m_rollerInputs = new RollerInputsAutoLogged();

  private SubsystemProfiles<ManipulatorState> m_profiles;

  public static enum ManipulatorState {
    kStow,
    kIntaking,
    kTuning,
    kScoring,
    kIdle,
  }

  public Manipulator(RollerIO rollerIO, WristIO wristIO) {
    m_wristIO = wristIO;
    m_rollerIO = rollerIO;
    Map<ManipulatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ManipulatorState.kStow, this::stowPeriodic);
    periodicHash.put(ManipulatorState.kTuning, this::tuningPeriodic);
    periodicHash.put(ManipulatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ManipulatorState.kIdle, this::idlePeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, ManipulatorState.kStow);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_wristIO.setPIDFF(
                0,
                ManipulatorWristConstants.wristkP.get(),
                ManipulatorWristConstants.wristkI.get(),
                ManipulatorWristConstants.wristkD.get(),
                ManipulatorWristConstants.wristkS.get());
          } else {
            m_wristIO.setPIDFF(
                0,
                ManipulatorWristConstants.wristkSimP.get(),
                ManipulatorWristConstants.wristkSimI.get(),
                ManipulatorWristConstants.wristkSimD.get(),
                ManipulatorWristConstants.wristkSimkS.get());
          }
        },
        ManipulatorWristConstants.wristkP,
        ManipulatorWristConstants.wristkI,
        ManipulatorWristConstants.wristkD,
        ManipulatorWristConstants.wristkS);

    m_wristIO.updateInputs(m_wristInputs);
    m_rollerIO.updateInputs(m_rollerInputs);

    if (ManipulatorRollerConstants.kTuningMode) {
      updateState(ManipulatorState.kTuning);
    }

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());
    Logger.processInputs("Manipulator/Wrist Inputs", m_wristInputs);
    Logger.processInputs("Manipulator/RollerInputs", m_rollerInputs);
    Logger.recordOutput("Manipulator/RunRollerScoring", m_runRollerScoring);
  }

  public void intakingPeriodic() {
    m_wristIO.setDesiredAngle(ManipulatorWristConstants.kIntakingAngle);
    m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerIntakingVoltage.get());
  }

  public void stowPeriodic() {
    m_wristIO.setDesiredAngle(ManipulatorWristConstants.kStowAngle);
    m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerStowVoltage.get());
  }

  public void idlePeriodic() {
    m_wristIO.setDesiredAngle(ManipulatorWristConstants.kStowAngle);
    m_rollerIO.setVoltage(0.0);
  }

  public void scoringPeriodic() {
    m_wristIO.setDesiredAngle(m_desiredAngle);
    if (m_runRollerScoring) {
      if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L1) {
        m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerL1ScoringVoltageManual.get());
      }
    } else if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L4) {
      m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerL4ScoringVoltage.get());
    } else if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L3
        || RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L2) {
      m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerL2L3ScoringVoltage.get());
    } else {
      m_rollerIO.setVoltage(0.0);
    }
  }

  public void tuningPeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(ManipulatorWristConstants.kTuningAngle.getAsDouble()));
    m_rollerIO.setVoltage(ManipulatorRollerConstants.kTuningVoltage.get());
  }

  public void updateState(ManipulatorState state) {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kTuning) {
      return;
    }
    m_profiles.setCurrentProfile(state);

    m_runRollerScoring = false;
  }

  public ManipulatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public Rotation2d getDesiredWristAngle() {
    return m_desiredAngle;
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromDegrees(m_wristInputs.currentAngleDeg);
  }

  public boolean atSetpoint() {
    return Math.abs(m_wristInputs.desiredAngleDeg - getCurrentAngle().getDegrees())
        < ManipulatorWristConstants.kWristTolerance;
  }

  public void runRollerScoring() {
    m_runRollerScoring = true;
  }

  public void stopRollerScoring() {
    m_runRollerScoring = false;
  }
}
