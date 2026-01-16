package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public Trigger coralIntake();

  public Trigger scoring();

  public Trigger coralOuttake();

  public Trigger setLocationL1();

  public Trigger setLocationL2();

  public Trigger setLocationL3();

  public Trigger setLocationL4();

  public Trigger zeroElevator();
}
