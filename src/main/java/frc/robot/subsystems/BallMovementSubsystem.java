package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import static frc.robot.Constants.ControllerConstants.*;

import org.ejml.dense.row.factory.DecompositionFactory_CDRM;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ExternalFollower;
// import edu.wpi.first.wpilibj.Compressor;

public class BallMovementSubsystem extends SubsystemBase {

    /* actuator instantiations */
    public final DoubleSolenoid m_ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            kBallPickupForwardChannel, kBallPickupReverseChannel);

    /* motor instantiations */
    public final TalonSRX m_intakeMotor = new TalonSRX(kIntakeMotor);

    /* sensor instantiations */

    public BallMovementSubsystem() {

    }

    protected boolean d_intakeLatch = false;// intake deployed status

    @Override
    public void periodic() {

        /* intake deploy latch switch mode */
        if (RobotContainer.m_manipulatorController.getRawButtonPressed(kA)) {
            d_intakeLatch = !d_intakeLatch;
        }
        if (d_intakeLatch) {
            m_ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            m_ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        if (d_intakeLatch) {
            m_intakeMotor.set(ControlMode.PercentOutput,
                    deadband(RobotContainer.m_manipulatorController.getRawAxis(kRightTrigger)));
        } else {
            m_intakeMotor.set(ControlMode.PercentOutput, 0);
        }

    }

    // throw small values
    private double deadband(double power) {
        if (power < 0.1 && power > -0.1) {
            return 0;
        }
        return power;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
