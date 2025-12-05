package frc.robot.Subsystems.manipulator.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ManipulatorWristConstants;

public class WristIOSim implements WristIO {
    private PIDController m_controller = new PIDController(0, 0, 0);
    private SingleJointedArmSim m_sim;

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(m_sim.getAngleRads());
    }

    private boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public WristIOSim() {
        m_sim = 
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                ManipulatorWristConstants.kWristSimGearbox,
                ManipulatorWristConstants.kWristSimGearing, 0),
            ManipulatorWristConstants.kWristArmLength, 0, 0, 0, 130, false, 0, null);
    m_controller.setTolerance(ManipulatorWristConstants.kWristTolerance);
    }

    
    @Override
    public void setDesiredAngle(Rotation2d angle) {
        m_controller.setSetpoint(angle.getDegrees());
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        double inputVoltage = m_controller.calculate(getCurrentAngle().getDegrees());
        m_sim.setInputVoltage(inputVoltage);
        inputs.currentAngleDeg = getCurrentAngle().getDegrees();
        inputs.velocityRPS = m_sim.getVelocityRadPerSec();
        inputs.desiredAngleDeg = m_controller.getSetpoint();
        inputs.voltage = inputVoltage;
        inputs.atSetpoint = atSetpoint();
        inputs.current = m_sim.getCurrentDrawAmps();
        }

    @Override
    public void setPIDFF(int slot, double kP, double kI, double kD, double kS) {
        m_controller.setPID(kP, kI, kD);
    }
}
