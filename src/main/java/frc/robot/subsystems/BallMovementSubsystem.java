package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import static frc.robot.Constants.ControllerConstants.*;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class BallMovementSubsystem extends SubsystemBase {

    // actuator instantiations
    public final DoubleSolenoid m_ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            kBallPickupForwardChannel, kBallPickupReverseChannel);
    public final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public BallMovementSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_compressor.enableDigital();
        if (RobotContainer.m_manipulatorController.getRawButtonPressed(kA)) {
            m_ballPickupSolenoid.toggle();

        }

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
