package frc.robot.Subsystems.manipulator.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristInputs {
    public double voltage;
    public double velocityRPS;
    public double current;
    public double desiredAngleDeg;
    public double currentAngleDeg;
    public double motorTemperature;
    public double statorCurrent;
    public boolean atSetpoint;
    public boolean motorIsConnected;
  }

  public void setDesiredAngle(Rotation2d angle);

  public void updateInputs(WristInputs inputs);

  public void setPIDFF(int slot, double kP, double kI, double kD, double kS);
}
