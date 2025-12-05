package frc.robot.Subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorWristConstants;
import frc.robot.Subsystems.manipulator.wrist.WristIO;

public class Manipulator extends SubsystemBase {
    private WristIO m_io;
    private Rotation2d m_desiredAngle = new Rotation2d();
    
    public static enum ManipulatorState {
      kStow,
      kIntaking,
      kIndexing,
      kScoring,
      kAlgaeDescoring,
      kAlgaeHold,
      kAlgaeOuttake,
      kCoralEject,
    }


@Override
public void periodic() {
  if(RobotBase.isReal()) {
    m_io.setPIDFF(
      0,
      ManipulatorWristConstants.kP.get(),
      ManipulatorWristConstants.kI.get(),
      ManipulatorWristConstants.kD.get(),
      ManipulatorWristConstants.kS.get());
} else {
  m_io.setPIDFF(
      0,
      ManipulatorWristConstants.kSimP.get(),
      ManipulatorWristConstants.kSimI.get(),
      ManipulatorWristConstants.kSimD.get(),
      ManipulatorWristConstants.kSimkS.get());
}
}

public void setDesiredAngle(Rotation2d angle) {
    m_desiredAngle = angle;
}

public Rotation2d getDesiredWristAngle() {
    return m_desiredAngle;
  }
}