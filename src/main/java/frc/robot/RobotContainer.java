package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Ports;
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
    } else {
      m_manipulator = new Manipulator(new RollerIOSim(), new WristIOSim());
      m_indexer = new Indexer(new IndexerIOSim());
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous configured! :(");
  }
}
