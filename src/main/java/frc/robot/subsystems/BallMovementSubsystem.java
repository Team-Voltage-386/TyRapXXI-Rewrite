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
    public final CANSparkMax m_launcherLead = new CANSparkMax(kLauncherLead, MotorType.kBrushless);
    public final CANSparkMax m_launcherFollower = new CANSparkMax(kLauncherFollower, MotorType.kBrushless);
    public final TalonSRX m_serializer = new TalonSRX(kSerializerMotor);
    public final TalonSRX m_feeder = new TalonSRX(kFeeder);
    /* sensor instantiations */

    public BallMovementSubsystem() {
        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.configNeutralDeadband(.1);

        m_launcherFollower.restoreFactoryDefaults();
        m_launcherLead.restoreFactoryDefaults();
        m_launcherFollower.follow(m_launcherLead, true);

        m_serializer.configFactoryDefault();
        m_serializer.configNeutralDeadband(.1);
        m_feeder.configFactoryDefault();
        m_feeder.configNeutralDeadband(.1);
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
        /* intake motor */
        if (d_intakeLatch) {
            m_intakeMotor.set(ControlMode.PercentOutput,
                    RobotContainer.m_manipulatorController.getRawAxis(kRightTrigger));
        } else {
            m_intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        /* launcher */
        if (d_intakeLatch) {
            m_launcherLead.set(RobotContainer.m_manipulatorController.getRawAxis(kLeftTrigger));
        } else {
            m_launcherLead.set(0);
        }
        /* serializer manual */
        if (d_intakeLatch) {
            m_serializer.set(ControlMode.PercentOutput,
                    RobotContainer.m_manipulatorController.getRawAxis(kLeftVertical));
        } else {
            m_serializer.set(ControlMode.PercentOutput, 0);
        }
        /* feeder manual */
        if (d_intakeLatch) {
            m_feeder.set(ControlMode.PercentOutput,
                    RobotContainer.m_manipulatorController.getRawAxis(kRightVertical));
        } else {
            m_feeder.set(ControlMode.PercentOutput, 0);
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
