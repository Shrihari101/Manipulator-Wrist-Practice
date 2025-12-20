package frc.robot.Subsystems.roller;

import edu.wpi.first.units.measure.Angle;

public class RollerIOReplay implements RollerIO {

  @Override
  public void updateInputs(RollerInputsAutoLogged rollerInputs) {}

  @Override
  public void setDesiredAngle(Angle position) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setPID(double kP, double kI, double kD, double kS) {}

  @Override
  public void setCurrentLimits(double currentLimits) {}
}
