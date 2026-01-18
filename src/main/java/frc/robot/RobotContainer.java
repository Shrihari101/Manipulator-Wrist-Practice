package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.ElevatorIOKraken;
import frc.robot.Subsystems.elevator.ElevatorIOSim;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.indexer.IndexerIOKraken;
import frc.robot.Subsystems.indexer.IndexerIOSim;
import frc.robot.Subsystems.manipulator.Manipulator;
import frc.robot.Subsystems.manipulator.roller.RollerIOKraken;
import frc.robot.Subsystems.manipulator.roller.RollerIOSim;
import frc.robot.Subsystems.manipulator.wrist.WristIOKraken;
import frc.robot.Subsystems.manipulator.wrist.WristIOSim;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;

public class RobotContainer {
  private Manipulator m_manipulator;
  private Indexer m_indexer;
  private Elevator m_elevator;
  private DriverControls m_driverControls;

  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  private void configureSubsystems() {

    if (RobotBase.isReal()) {
      m_manipulator =
          new Manipulator(
              new RollerIOKraken(Ports.kManipulatorRoller),
              new WristIOKraken(Ports.kManipulatorWrist, Ports.kManipulatorAbsoluteEncoder));
      m_indexer = new Indexer(new IndexerIOKraken(Ports.kIndexerSideMotor, Ports.kIndexerTopMotor));
      m_elevator =
          new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));
    } else {
      m_manipulator = new Manipulator(new RollerIOSim(), new WristIOSim());
      m_indexer = new Indexer(new IndexerIOSim());
      m_elevator = new Elevator(new ElevatorIOSim());
    }
  }

  private void configureCommands() {
    RobotState.startInstance(m_indexer, m_manipulator, m_elevator);
  }

  private void configureControllers() {
    // m_driverControls = new DriverControlsPS5(0);
    m_driverControls = new DriverControlsXbox(0);
  }

  private void configureButtonBindings() {

    m_driverControls
        .setLocationL1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL1);
                }));

    m_driverControls
        .setLocationL2()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL2);
                }));

    m_driverControls
        .setLocationL3()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL3);
                }));

    m_driverControls
        .setLocationL4()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL4);
                }));

    m_driverControls
        .coralOuttake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kOuttaking);
                }));

    m_driverControls
        .scoring()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kScoring);
                }));
    m_driverControls
        .coralIntake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kIntaking);
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous configured! :(");
  }
}
