package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BallMovementConstants.*;
import static frc.robot.Constants.ControllerConstants.*;

import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class BallMovementSubsystem extends SubsystemBase {

    /* actuator instantiations */
    public final DoubleSolenoid ballPickupSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            kBallPickupForwardChannel, kBallPickupReverseChannel);

    /* motor instantiations */
    public final TalonSRX intakeMotor = new TalonSRX(kIntakeMotor);
    public final CANSparkMax launcherLeadMotor = new CANSparkMax(kLauncherLead, MotorType.kBrushless);
    public final CANSparkMax launcherFollowerMotor = new CANSparkMax(kLauncherFollower, MotorType.kBrushless);
    public final TalonSRX serializerMotor = new TalonSRX(kSerializerMotor);
    public final TalonSRX feederMotor = new TalonSRX(kFeeder);
    public final DigitalInput indexerSensor = new DigitalInput(kIndexer);

    I2C.Port entranceSensorI2CPort = I2C.Port.kOnboard; // Port 0
    I2C.Port feederSensorI2CPort = I2C.Port.kMXP; // Port 1
    /* sensor instantiations */
    public final ColorSensorV3 entranceColorSensor = new ColorSensorV3(entranceSensorI2CPort);;
    public final ColorSensorV3 feederColorSensor = new ColorSensorV3(feederSensorI2CPort);
    boolean entranceBallDetected;
    boolean feederBallDetected;

    // Creates a Shuffleboard tab for the ball movement subsystem
    private ShuffleboardTab tab = Shuffleboard.getTab("Ball Movement");

    // Create sensor widgets
    private NetworkTableEntry feedWidget = tab.add("Feeder Sensor", false).withPosition(7, 3).withSize(2, 1).getEntry();
    private NetworkTableEntry indexWidget = tab.add("Indexer", false).withPosition(5, 3).withSize(2, 1).getEntry();
    private NetworkTableEntry entranceWidget = tab.add("Entrance", false).withPosition(3, 3).withSize(2, 1).getEntry();

    private NetworkTableEntry entranceColorSensorProximityWidget = tab.add("Entrance Color Sensor - Proximity", 2048)
            .withSize(2, 1).getEntry();

    private NetworkTableEntry feederColorSensorProximityWidget = tab.add("Feeder Color Sensor - Proximity", 2048)
            .withSize(2, 1).getEntry();

    public BallMovementSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(.1);

        launcherFollowerMotor.restoreFactoryDefaults();
        launcherLeadMotor.restoreFactoryDefaults();
        launcherFollowerMotor.follow(launcherLeadMotor, true);

        serializerMotor.configFactoryDefault();
        serializerMotor.configNeutralDeadband(.1);
        feederMotor.configFactoryDefault();
        feederMotor.configNeutralDeadband(.1);
    }

    public boolean getEntranceSensor() {
        return entranceBallDetected;
    }

    public boolean getIndexerSensor() {
        return !indexerSensor.get();
    }

    public boolean getFeederSensor() {
        return feederBallDetected;
    }

    @Override
    public void periodic() {

        // Update sensor widgets
        indexWidget.setBoolean(getIndexerSensor());
        entranceWidget.setBoolean(getEntranceSensor());
        feedWidget.setBoolean(getFeederSensor());

        // Update color sensor widgets
        entranceColorSensorProximityWidget.setNumber(entranceColorSensor.getProximity());
        feederColorSensorProximityWidget.setNumber(feederColorSensor.getProximity());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
