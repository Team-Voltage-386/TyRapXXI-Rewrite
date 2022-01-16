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
    public final DoubleSolenoid ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            kBallPickupForwardChannel, kBallPickupReverseChannel);

    /* motor instantiations */
    public final TalonSRX intakeMotor = new TalonSRX(kIntakeMotor);
    public final CANSparkMax launcherLead = new CANSparkMax(kLauncherLead, MotorType.kBrushless);
    public final CANSparkMax launcherFollower = new CANSparkMax(kLauncherFollower, MotorType.kBrushless);
    public final TalonSRX serializer = new TalonSRX(kSerializerMotor);
    public final TalonSRX feeder = new TalonSRX(kFeeder);
    /* sensor instantiations */

    public BallMovementSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(.1);

        launcherFollower.restoreFactoryDefaults();
        launcherLead.restoreFactoryDefaults();
        launcherFollower.follow(launcherLead, true);

        serializer.configFactoryDefault();
        serializer.configNeutralDeadband(.1);
        feeder.configFactoryDefault();
        feeder.configNeutralDeadband(.1);
    }

    protected boolean intakeLatch = false;// intake deployed status

    @Override
    public void periodic() {

        /* intake deploy latch switch mode */
        if (RobotContainer.manipulatorController.getRawButtonPressed(kA)) {
            intakeLatch = !intakeLatch;
        }
        if (intakeLatch) {
            ballPickupSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            ballPickupSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        /* intake motor */
        if (intakeLatch) {
            intakeMotor.set(ControlMode.PercentOutput,
                    RobotContainer.manipulatorController.getRawAxis(kRightTrigger));
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        /* launcher */
        if (intakeLatch) {
            launcherLead.set(RobotContainer.manipulatorController.getRawAxis(kLeftTrigger));
        } else {
            launcherLead.set(0);
        }
        /* serializer manual */
        if (intakeLatch) {
            serializer.set(ControlMode.PercentOutput,
                    RobotContainer.manipulatorController.getRawAxis(kLeftVertical));
        } else {
            serializer.set(ControlMode.PercentOutput, 0);
        }
        /* feeder manual */
        if (intakeLatch) {
            feeder.set(ControlMode.PercentOutput,
                    RobotContainer.manipulatorController.getRawAxis(kRightVertical));
        } else {
            feeder.set(ControlMode.PercentOutput, 0);
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
