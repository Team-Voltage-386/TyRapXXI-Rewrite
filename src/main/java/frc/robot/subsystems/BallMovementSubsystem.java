package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import frc.robot.Constants.ShooterData;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.pidConstants.*;

public class BallMovementSubsystem extends SubsystemBase {

    // actuator instantiations 
    private final DoubleSolenoid ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kBallPickupForwardChannel, kBallPickupReverseChannel);

    // motor instantiations 
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeMotor);
    private final CANSparkMax launcherLeadMotor = new CANSparkMax(kLauncherLead, MotorType.kBrushless);
    private final CANSparkMax launcherFollowerMotor = new CANSparkMax(kLauncherFollower, MotorType.kBrushless);
    private final TalonSRX serializerMotor = new TalonSRX(kSerializerMotor);
    private final TalonSRX feederMotor = new TalonSRX(kFeeder);
    private final DigitalInput indexerSensor = new DigitalInput(kIndexer);
      
    // sensor instantiations 
    public double entranceP = 0;
    public double feederP = 0;
    /**Entrance sensor is tripped*/
    public Boolean entrance;
    /**Feeder sensor is tripped*/
    public Boolean feed = false;
    /**Indexer sensor is tripped*/
    public Boolean index = false;
    public Boolean drumControllerOn = false;

    private Boolean hoodLowLimit = false;
    public double hoodPosition = 0;
    public double hoodSet = 0;
    private Boolean calibrated = false;
    private PIDController pidH = new PIDController(HP,HI,HD);
    private PIDController pidL = new PIDController(LP,LI,LD);
    public double drumCurrentSpeed = 0;
    public Boolean drumIdle = false;
    private Boolean drumPIDRunning = false;
    public int drumSP = 0;
    public Boolean autoSF = false;

    private final ShuffleboardTab _tab;
    private final NetworkTableEntry hpWidget;
    private final NetworkTableEntry hsWidget;
    private final NetworkTableEntry dSWidget;

    /**Creates a BallMovementSubsystem*/
    public BallMovementSubsystem(ShuffleboardTab t) {
        _tab = t;
        hpWidget = _tab.add("HoodPosition",0).withSize(1, 1).withPosition(0, 0).getEntry();
        hsWidget = _tab.add("HoodSet",0).withSize(1,1).withPosition(1, 0).getEntry();
        dSWidget = _tab.add("DrumSpeed",0).withSize(2,1).withPosition(3, 3).getEntry();
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
    
    /** Directly set launcher power
     * @param power Launching Power
     */
    public void setDrumPower(double power) {
        launcherLeadMotor.set(power);
    }

    /** set feeder on/off
     * @param on feeder on/off
     */
    public void runFeeder(Boolean on) {
        if (on) feederMotor.set(ControlMode.PercentOutput, feederPower);
        else feederMotor.set(ControlMode.PercentOutput, 0);
    }

    public void runFeedSlow(Boolean on) {
        if (on) feederMotor.set(ControlMode.PercentOutput, 0.2);
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

    /**run the hood control system*/
    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(-1*pidH.calculate(hoodPosition, hoodSet), -1*HC, HC);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);
            else hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, -1, 0));
        }
    }

    /**recalibrate pids and hood*/
    public void reCalibrate() {
        pidH.reset();
        pidL.reset();
        calibrated = false;
    }

    public Boolean RTF() {
        double Hdiff = Math.abs(hoodSet - hoodPosition);
        double Ddiff = Math.abs(drumSP - drumCurrentSpeed);
        return (Ddiff < launcherSpeedTolerances) && (Hdiff < hoodTolerance);
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
            runHood();
            hoodPosition = hoodEncoder.get();
        }

        if (drumControllerOn) {
            drumPIDRunning = true;
            setDrumPower(pidL.calculate(drumCurrentSpeed, drumSP));
        } else if (drumIdle) {
            drumPIDRunning = true;
            setDrumPower(pidL.calculate(drumCurrentSpeed,drumIdleSpeed));
        }else {
            setDrumPower(0);
            if (drumPIDRunning) {
                drumPIDRunning = false;
                pidL.reset();
            }
        }

        runSerializer(!feed && autoSF);
        runFeedSlow(!feed && autoSF);

        drumCurrentSpeed = launcherLeadMotor.getEncoder().getVelocity();
        hpWidget.setDouble(hoodPosition);
        hsWidget.setDouble(hoodSet);
        dSWidget.setDouble(drumCurrentSpeed);
        entranceP = entranceSensor.getProximity();
        feederP = feederSensor.getProximity();
        index = !indexerSensor.get();
        feed = feederP >= kFeederProximityThreshold;
        entrance = entranceP >= kEntranceProximityThreshold;
    }

    public void stop() {
        setDrumPower(0);
        drumControllerOn = false;
        drumPIDRunning = false;
        runFeeder(false);
        runIntake(false);
        runSerializer(false);
    }

    public void setAimDistance(double m) {
        int i = -1;
        for (int j = 0; j < ShooterData.distances.length; j++) {
            if (m < ShooterData.distances[j]) {
                i = j;
                return;
            }
        }
        double upper = ShooterData.distances[i];
        double lower = ShooterData.distances[i-1];
        double lerpFactor = (m-lower)/(upper-lower);
        upper = ShooterData.drumSpeeds[i];
        lower = ShooterData.drumSpeeds[i-1];
        drumSP = (int)(lower + ((upper-lower)*lerpFactor));
        upper = ShooterData.hoodPositions[i];
        lower = ShooterData.hoodPositions[i-1];
        hoodSet = lower + ((upper-lower)*lerpFactor);
    }
}