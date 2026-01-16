package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {

  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger coralIntake() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger coralOuttake() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger setLocationL1() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setLocationL2() {
    return m_controller.povRight();
  }

  @Override
  public Trigger setLocationL3() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setLocationL4() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger zeroElevator() {
    return m_controller.a();
  }
}
