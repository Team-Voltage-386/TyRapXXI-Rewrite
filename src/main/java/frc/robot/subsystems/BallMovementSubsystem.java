package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallMovementConstants;

import static frc.robot.Constants.BallMovementConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.pidConstants.*;

public class BallMovementSubsystem extends SubsystemBase {

    // actuator instantiations 
    public final DoubleSolenoid ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kBallPickupForwardChannel, kBallPickupReverseChannel);

    // motor instantiations 
    public final TalonSRX intakeMotor = new TalonSRX(kIntakeMotor);
    public final CANSparkMax launcherLeadMotor = new CANSparkMax(kLauncherLead, MotorType.kBrushless);
    public final CANSparkMax launcherFollowerMotor = new CANSparkMax(kLauncherFollower, MotorType.kBrushless);
    public final TalonSRX serializerMotor = new TalonSRX(kSerializerMotor);
    public final TalonSRX feederMotor = new TalonSRX(kFeeder);
    public final DigitalInput indexerSensor = new DigitalInput(kIndexer);
      
    // sensor instantiations 
    public double entranceP = 0;
    public double feederP = 0;
    /**Entrance sensor is tripped*/
    public Boolean entrance = entranceP >= kEntranceProximityThreshold;
    /**Feeder sensor is tripped*/
    public Boolean feed = feederP >= kFeederProximityThreshold;
    /**Indexer sensor is tripped*/
    public Boolean index = false;

    public Boolean hoodLowLimit = false;
    public double hoodPosition = 0;
    public Boolean calibrated = false;
    public PIDController pidH = new PIDController(HP,HI,HD);
    public PIDController pidL = new PIDController(LP,LI,LD);

    public double launcherCurrentSpeed = 0;

    /**Creates a BallMovementSubsystem*/
    public BallMovementSubsystem() {
        pidL.reset();
        pidL.setTolerance(1,1);
        pidH.reset();
        pidH.setTolerance(1,1);

        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);

        launcherFollowerMotor.restoreFactoryDefaults();
        launcherLeadMotor.restoreFactoryDefaults();
        launcherFollowerMotor.follow(launcherLeadMotor, true);

        serializerMotor.configFactoryDefault();
        serializerMotor.configNeutralDeadband(0);
        feederMotor.configFactoryDefault();
        feederMotor.configNeutralDeadband(0);

        hoodMotor.configFactoryDefault();
        hoodMotor.configNeutralDeadband(0);
        calibrated = false;
    }

    /**Operate Ball Intake
     * @param deploy True: deploy, False: retract
    */
    public void deployIntake(Boolean deploy) {
      if (deploy) ballPickupSolenoid.set(kIntakeDeployed);
      else ballPickupSolenoid.set(kIntakeRetracted);
    }
    
    /** Set launcher power
     * @param power Launching Power
     */
    public void setLauncherPower(double power) {
        launcherLeadMotor.set(power);
    }

    public void setLauncherOn(Boolean b) {
        if (b) launcherLeadMotor.set(pidL.calculate(launcherCurrentSpeed,launcherSpeedSet));
        else  launcherLeadMotor.set(0);
    }

    /** set feeder on/off
     * @param on feeder on/off
     */
    public void runFeeder(Boolean on) {
        if (on) feederMotor.set(ControlMode.PercentOutput, feederPower);
        else feederMotor.set(ControlMode.PercentOutput, 0);
    }

    /** set serializer motor on/off
     *@param on motor on/off
     */
    public void runSerializer(Boolean on) {
        if (on) serializerMotor.set(ControlMode.PercentOutput, serializerPower);
        else serializerMotor.set(ControlMode.PercentOutput, 0);
    }

    /** set intake motor on/off
     * @param on motor on/off
     */
    public void runIntake(Boolean on) {
        if (on) intakeMotor.set(ControlMode.PercentOutput, intakePower);
        else intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setHoodPosition(double p) {
        if (calibrated) {
            double control = MathUtil.clamp(-1*pidH.calculate(hoodPosition, MathUtil.clamp(p,0,0.7)), -1*HC, HC);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);
            else hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, -1, 0));
        }
    }

    public void reCalibrate() {
        pidH.reset();
        pidL.reset();
        calibrated = false;
    }

    @Override
    public void periodic() {
        hoodLowLimit = !hoodLimit.get();
        if (!calibrated) {
            hoodMotor.set(ControlMode.PercentOutput, 1);
            if (hoodLowLimit) {
                hoodEncoder.reset();
                calibrated = true;
            }
        } else {
            launcherCurrentSpeed = launcherLeadMotor.getEncoder().getVelocity();
            entranceP = entranceSensor.getProximity();
            feederP = feederSensor.getProximity();
            index = !indexerSensor.get();
            hoodPosition = hoodEncoder.get();
        }
    }
}