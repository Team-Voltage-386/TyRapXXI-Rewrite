package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
 
    I2C.Port entranceSensorI2CPort = I2C.Port.kOnboard; // Port 0
    I2C.Port feederSensorI2CPort = I2C.Port.kMXP; // Port 1  
    // sensor instantiations 
    private final ColorSensorV3 entranceSensor = new ColorSensorV3(entranceSensorI2CPort);
    private final ColorSensorV3 feederSensor = new ColorSensorV3(feederSensorI2CPort);
    public double entranceP = entranceSensor.getProximity();
    public double feederP = feederSensor.getProximity();
    /**Entrance sensor is tripped*/
    public Boolean entrance = entranceP >= kEntranceProximityThreshold;
    /**Feeder sensor is tripped*/
    public Boolean feed = feederP >= kFeederProximityThreshold;
    /**Indexer sensor is tripped*/
    public Boolean index = !indexerSensor.get();

    /**Creates a BallMovementSubsystem*/
    public BallMovementSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);

        launcherFollowerMotor.restoreFactoryDefaults();
        launcherLeadMotor.restoreFactoryDefaults();
        launcherFollowerMotor.follow(launcherLeadMotor, true);

        serializerMotor.configFactoryDefault();
        serializerMotor.configNeutralDeadband(0);
        feederMotor.configFactoryDefault();
        feederMotor.configNeutralDeadband(0);
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
}