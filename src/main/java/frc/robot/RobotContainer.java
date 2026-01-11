package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.Elevator.ElevatorState;
import frc.robot.Subsystems.elevator.ElevatorIOKraken;
import frc.robot.Subsystems.elevator.ElevatorIOSim;
import frc.robot.Subsystems.indexer.Indexer;
import frc.robot.Subsystems.indexer.Indexer.IndexerState;
import frc.robot.Subsystems.indexer.IndexerIOKraken;
import frc.robot.Subsystems.indexer.IndexerIOSim;
import frc.robot.Subsystems.manipulator.Manipulator;
import frc.robot.Subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.Subsystems.manipulator.roller.RollerIOKraken;
import frc.robot.Subsystems.manipulator.roller.RollerIOSim;
import frc.robot.Subsystems.manipulator.wrist.WristIOKraken;
import frc.robot.Subsystems.manipulator.wrist.WristIOSim;

public class RobotContainer {
  private Manipulator m_manipulator;
  private Indexer m_indexer;
  private Elevator m_elevator;
  private CommandPS5Controller m_controller;

  public RobotContainer() {
    configureSubsystems();
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

  private void configureControllers() {
    m_controller = new CommandPS5Controller(0);
  }

  private void configureButtonBindings() {

    m_controller
        .R2()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_manipulator.updateState(ManipulatorState.kStow);
                }));
    m_controller
        .R1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_manipulator.updateState(ManipulatorState.kIntaking);
                }));
    m_controller
        .L1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_indexer.updateState(IndexerState.kStop);
                }));
    m_controller
        .L2()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_indexer.updateState(IndexerState.kIndexing);
                }));
    m_controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.updateState(ElevatorState.kStow);
                }));
    m_controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.updateState(ElevatorState.kTuning);
                }));
    m_controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_elevator.getCurrentState() == ElevatorState.kScoring) {
                    m_elevator.setDesiredHeight(0);
                    m_elevator.updateState(ElevatorState.kIntaking);
                  } else {
                    m_elevator.updateState(ElevatorState.kScoring);
                  }
                }));
    m_controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_elevator.getCurrentState() == ElevatorState.kScoring) {
                    m_elevator.setDesiredHeight(0);
                    m_elevator.updateState(ElevatorState.kStow);
                  } else {
                    m_elevator.updateState(ElevatorState.kScoring);
                  }
                }));
    m_controller
        .cross()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL1);
                }));
    m_controller
        .circle()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL2);
                }));
    m_controller
        .triangle()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL3);
                }));
    m_controller
        .square()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.setDesiredHeight(ElevatorConstants.kL4);
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous configured! :(");
  }
}
