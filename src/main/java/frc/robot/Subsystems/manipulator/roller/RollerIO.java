package frc.robot.Subsystems.manipulator.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerInputs {
    public boolean motorIsConnected;
    public double temperature;
    public double current;
    public double statorCurrent;
    public double velocity;
    public double acceleration;
    public double voltage;
  }

  public void updateInputs(RollerInputsAutoLogged rollerInputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double currentLimits);
}
