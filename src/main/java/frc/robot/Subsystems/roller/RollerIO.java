package frc.robot.Subsystems.roller;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerInputs {
    public boolean motorIsConnected;
    public double temperature;
    public double current;
    public double statorCurrent;
    public double desiredAngle;
    public double currentAngle;
    public boolean positionControl;
    public double velocity;
    public double acceleration;
    public double voltage;
  }

  public void updateInputs(RollerInputsAutoLogged rollerInputs);

  public void setDesiredAngle(Angle position);

  public void setVoltage(double voltage);

  public void setPID(double kP, double kI, double kD, double kS);

  public void setCurrentLimits(double currentLimits);
}
