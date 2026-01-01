package frc.robot.Subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public boolean atSetpoint;
    public boolean isLeadingMotorConnected;
    public boolean isFollowingMotorConnected;
    public boolean positionControl;
    public double desiredLocation;
    public double leadingVelocity;
    public double followingVelocity;
    public double leadingAcceleration;
    public double followingAcceleration;
    public double leadingVoltage;
    public double followingVoltage;
    public double leadingSupplyCurrent;
    public double followingSupplyCurrent;
    public double leadingStatorCurrent;
    public double followingStatorCurrent;
    public double leadingPosition;
    public double followingPosition;
    public double leadingTemperature;
    public double followingTemperature;
  }

  public void updateInputs(ElevatorInputs inputs);

  public void setDesiredHeight(double inchesHeight);

  public void setPIDFF(int slot, double kP, double kI, double kD, double kG);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double currentLimits);
}
