package frc.robot.Subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Wrist extends SubsystemBase {
    private WristIO m_io;
    private Rotation2d m_desiredAngle = new Rotation2d();
    
    public Wrist (WristIO wristIO) {
        m_io = wristIO;
    }

@Override
public void periodic() {
  if(RobotBase.isReal()) {
    m_io.setPIDFF(
      0,
      ManipulatorConstants.kP.get(),
      ManipulatorConstants.kI.get(),
      ManipulatorConstants.kD.get(),
      ManipulatorConstants.kS.get());
} else {
  m_io.setPIDFF(
      0,
      ManipulatorConstants.kSimP.get(),
      ManipulatorConstants.kSimI.get(),
      ManipulatorConstants.kSimD.get(),
      ManipulatorConstants.kSimkS.get());
}
}

public void setDesiredAngle(Rotation2d angle) {
    m_desiredAngle = angle;
}

public Rotation2d getDesiredWristAngle() {
    return m_desiredAngle;
  }
}