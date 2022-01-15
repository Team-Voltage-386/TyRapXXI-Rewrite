package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import static frc.robot.Constants.ControllerConstants.*;

import org.ejml.dense.row.factory.DecompositionFactory_CDRM;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Compressor;

public class BallMovementSubsystem extends SubsystemBase {

    // actuator instantiations
    public final DoubleSolenoid m_ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            kBallPickupForwardChannel, kBallPickupReverseChannel);
    // public final Compressor m_compressor = new
    // Compressor(PneumaticsModuleType.CTREPCM);

    public BallMovementSubsystem() {

    }

    protected boolean d_intakeLatch = false;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /* Toggle mode */
        // if (RobotContainer.m_manipulatorController.getRawButtonPressed(kA)) {
        // if (m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kReverse
        // || m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kOff) {
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        // System.out.println("deploy");
        // } else {
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        // System.out.println("retract");
        // }
        // }

        /* A-B mode */
        // if (RobotContainer.m_manipulatorController.getRawButtonPressed(kA)
        // && (m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kReverse
        // || m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kOff)) {
        // System.out.println("!!!!!!about to deploy!!!!!!");
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        // System.out.println("!!!!!!deploy!!!!!!");
        // } else if (RobotContainer.m_manipulatorController.getRawButtonPressed(kB)
        // && (m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kForward
        // || m_ballPickupSolenoid.get() == DoubleSolenoid.Value.kOff)) {
        // System.out.println("!!!!!!about to retract!!!!!!");
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        // System.out.println("!!!!!!retract!!!!!!");
        // }

        /* latch switch mode */
        if (RobotContainer.m_manipulatorController.getRawButtonPressed(kA)) {
            d_intakeLatch = !d_intakeLatch;
        }

        if (d_intakeLatch) {
            m_ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            m_ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        /* baby a-b mode */
        // if(RobotContainer.m_manipulatorController.getRawButtonPressed(kA)){
        // System.out.println("A");
        // }
        // else if(RobotContainer.m_manipulatorController.getRawButtonPressed(kB)){
        // System.out.println("B");
        // }
        // System.out.println(m_ballPickupSolenoid.get());

        // if
        // (RobotContainer.m_manipulatorController.getRawButton(kBallPickupForwardChannel))
        // {
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        // } else if
        // (RobotContainer.m_manipulatorController.getRawButton(kBallPickupReverseChannel))
        // {
        // m_ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
