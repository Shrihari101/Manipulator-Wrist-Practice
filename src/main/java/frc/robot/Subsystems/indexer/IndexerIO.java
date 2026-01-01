package frc.robot.Subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double sidePosition;
    public double sideVelocity;
    public double sideCurrent;
    public double sideStatorCurrent;
    public double sideVoltage;
    public double sideTemperature;
    public boolean sideMotorIsConnected;
    public double topPosition;
    public double topVelocity;
    public double topCurrent;
    public double topStatorCurrent;
    public double topVoltage;
    public double topTemperature;
    public boolean topMotorIsConnected;
  }

  public void setVoltage(double sideVoltage, double topVoltage);

  public void updateInputs(IndexerInputs inputs);

  public void setCurrentLimits(double currentLimits);
}
