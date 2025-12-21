package frc.robot.Subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorRollerConstants;
import frc.robot.Constants.ManipulatorWristConstants;
import frc.robot.Subsystems.manipulator.wrist.WristIO;
import frc.robot.Subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.Subsystems.roller.RollerIO;
import frc.robot.Subsystems.roller.RollerInputsAutoLogged;
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
  private boolean m_runRollerAlgaeDescoring = false;
  private boolean m_runRollerBargeScoring = false;
  private boolean m_rollerPositionControlSet = false;

  public final WristInputsAutoLogged m_wristInputs = new WristInputsAutoLogged();
  public final RollerInputsAutoLogged m_rollerInputs = new RollerInputsAutoLogged();

  private SubsystemProfiles<ManipulatorState> m_profiles;

  public static enum ManipulatorState {
    kStow,
    kIntaking,
    kTuning,
  }

  public Manipulator(RollerIO rollerIO, WristIO wristIO) {
    m_wristIO = wristIO;
    m_rollerIO = rollerIO;
    Map<ManipulatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ManipulatorState.kStow, this::stowPeriodic);
    periodicHash.put(ManipulatorState.kTuning, this::tuningPeriodic);
    periodicHash.put(ManipulatorState.kIntaking, this::intakingPeriodic);

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

            m_rollerIO.setPID(
                ManipulatorRollerConstants.rollerkP.get(),
                ManipulatorRollerConstants.rollerkI.get(),
                ManipulatorRollerConstants.rollerkD.get(),
                ManipulatorRollerConstants.rollerkS.get());
          } else {
            m_wristIO.setPIDFF(
                0,
                ManipulatorWristConstants.wristkSimP.get(),
                ManipulatorWristConstants.wristkSimI.get(),
                ManipulatorWristConstants.wristkSimD.get(),
                ManipulatorWristConstants.wristkSimkS.get());

            m_rollerIO.setPID(
                ManipulatorRollerConstants.rollerkSimP.get(),
                ManipulatorRollerConstants.rollerkSimI.get(),
                ManipulatorRollerConstants.rollerkSimD.get(),
                ManipulatorRollerConstants.rollerkSimkS.get());
          }
        },
        ManipulatorWristConstants.wristkP,
        ManipulatorWristConstants.wristkI,
        ManipulatorWristConstants.wristkD,
        ManipulatorWristConstants.wristkS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_rollerIO.setPID(
                ManipulatorRollerConstants.rollerkP.get(),
                ManipulatorRollerConstants.rollerkI.get(),
                ManipulatorRollerConstants.rollerkD.get(),
                ManipulatorRollerConstants.rollerkS.get());
          } else {
            m_rollerIO.setPID(
                ManipulatorRollerConstants.rollerkSimP.get(),
                ManipulatorRollerConstants.rollerkSimI.get(),
                ManipulatorRollerConstants.rollerkSimD.get(),
                ManipulatorRollerConstants.rollerkSimkS.get());
          }
        },
        ManipulatorRollerConstants.rollerkP,
        ManipulatorRollerConstants.rollerkI,
        ManipulatorRollerConstants.rollerkD,
        ManipulatorRollerConstants.rollerkS);

    m_wristIO.updateInputs(m_wristInputs);
    m_rollerIO.updateInputs(m_rollerInputs);

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());
    Logger.processInputs("Manipulator/Wrist Inputs", m_wristInputs);
    Logger.processInputs("Manipulator/RollerInputs", m_rollerInputs);
  }

  public void intakingPeriodic() {
    m_wristIO.setDesiredAngle(ManipulatorWristConstants.kIntakingAngle);
    m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerIntakingVoltage.get());
  }

  public void stowPeriodic() {
    m_wristIO.setDesiredAngle(ManipulatorWristConstants.kStowAngle);
    m_rollerIO.setVoltage(ManipulatorRollerConstants.kRollerStowVoltage.get());
  }

  public void tuningPeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(ManipulatorWristConstants.kTuningAngle.getAsDouble()));
  }

  public void updateState(ManipulatorState state) {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kTuning) {
      return;
    }
    m_runRollerScoring = false;
    m_runRollerAlgaeDescoring = true;
    m_runRollerBargeScoring = false;
    m_rollerPositionControlSet = false;

    m_profiles.setCurrentProfile(state);
  }

  public ManipulatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredAngle(Rotation2d angle) {
    m_desiredAngle = angle;
  }

  public Rotation2d getDesiredWristAngle() {
    return m_desiredAngle;
  }

  public Angle getRollerPosition() {
    return Degrees.of(m_rollerInputs.currentAngle);
  }
}
