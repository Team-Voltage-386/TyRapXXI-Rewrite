package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**Carl's wacky dash*/
public class Dashboard {
    // Creates a shuffleboard tab
    private static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Dashboard");
    private static ShuffleboardTab llTab = Shuffleboard.getTab("LL-AutoAim");
    
    // Diagnostics in:
    private static DriveSubsystem _driveSS;
    private static LLSubsystem _llSS;
    private static RobotContainer _rc;
    // Create output widgets
    private static NetworkTableEntry frontLeftOutputWidget = driverTab.add("F-L Output", 0).withPosition(0, 0).getEntry();
    private static NetworkTableEntry frontRightOutputWidget = driverTab.add("F-R Output", 0).withPosition(1, 0).getEntry();
    private static NetworkTableEntry backLeftOutputWidget = driverTab.add("B-L Output", 0).withPosition(0, 1).getEntry();
    private static NetworkTableEntry backRightOutputWidget = driverTab.add("B-R Output", 0).withPosition(1, 1).getEntry();
    // Create temperature widgets
    private static NetworkTableEntry frontLeftTempWidget = driverTab.add("F-L Temp", 0).withPosition(3, 0).getEntry();
    private static NetworkTableEntry frontRightTempWidget = driverTab.add("F-R Temp", 0).withPosition(4, 0).getEntry();
    private static NetworkTableEntry backLeftTempWidget = driverTab.add("B-L Temp", 0).withPosition(3, 1).getEntry();
    private static NetworkTableEntry backRightTempWidget = driverTab.add("B-R Temp", 0).withPosition(4, 1).getEntry();
    // Create current widgets
    private static NetworkTableEntry frontLeftCurrentWidget = driverTab.add("F-L Current", 0).withPosition(6, 0).getEntry();
    private static NetworkTableEntry frontRightCurrentWidget = driverTab.add("F-R Current", 0).withPosition(7, 0).getEntry();
    private static NetworkTableEntry backLeftCurrentWidget = driverTab.add("B-L Current", 0).withPosition(6, 1).getEntry();
    private static NetworkTableEntry backRightCurrentWidget = driverTab.add("B-R Current", 0).withPosition(7, 1).getEntry();
    // Create encoder widgets
    private static NetworkTableEntry leftEncoderWidget = driverTab.add("Left Encoder", 0).withSize(2, 1).withPosition(2, 2).getEntry();
    private static NetworkTableEntry rightEncoderWidget = driverTab.add("Right Encoder", 0).withSize(2, 1).withPosition(4, 2).getEntry();
    //DriveSystemsOut:
    private static SendableChooser<Command> driveModeChooser = new SendableChooser<Command>();

    private static NetworkTableEntry llaWidget = llTab.add("LL-AutoAim Active",false).withSize(1,1).withPosition(0,0).getEntry();
    private static NetworkTableEntry lltxWidget = llTab.add("Process Variable Error",0).withSize(2,1).withPosition(0,1).getEntry();
    private static NetworkTableEntry lltoWidget = llTab.add("PID Output",0).withSize(2,1).withPosition(0,2).getEntry();
    private static NetworkTableEntry lltvWidget = llTab.add("Target Found",false).withSize(1,1).withPosition(1, 0).getEntry();
    private static NetworkTableEntry llpWidget = llTab.add("P: ",Constants.pidConstants.LLP).withSize(1,1).withPosition(0,3).getEntry();
    private static NetworkTableEntry lldWidget = llTab.add("D: ", Constants.pidConstants.LLD).withSize(1,1).withPosition(1,3).getEntry();

    public static Boolean commandChange = false;
    //Manual Command Selection:
    public static Command manualC;
    private static double LLP = Constants.pidConstants.LLP;
    private static double LLD = Constants.pidConstants.LLD;
    //Autonomous Command Selection: 

    /**
     *  Initialize the dash, customize at will
     */
    public static void init() {
        _rc = Robot.m_robotContainer;
        _llSS = _rc.limeLightSubsystem;
        //Instantiate DriveModeChooser:
        _driveSS = _rc.driveSubSystem;
        driveModeChooser.setDefaultOption("ArcadeDrive", _rc.manualDriveArcade);
        driveModeChooser.addOption("TankDrive", _rc.manualDriveTank);
        driverTab.add("Drive Mode", driveModeChooser);
        manualC = driveModeChooser.getSelected();
    }

    /**
     * Update the dash, customize at will
     */
    public static void update() {
        //DriverTab:
        // Update output widgets
        frontLeftOutputWidget.setDouble(_driveSS.frontLeftMotor.get());
        frontRightOutputWidget.setDouble(_driveSS.frontRightMotor.get());
        backLeftOutputWidget.setDouble(_driveSS.rearLeftMotor.get());
        backRightOutputWidget.setDouble(_driveSS.rearRightMotor.get());
        // Update temp widgets
        frontLeftTempWidget.setDouble(_driveSS.frontLeftMotor.getMotorTemperature());
        frontRightTempWidget.setDouble(_driveSS.frontRightMotor.getMotorTemperature());
        backLeftTempWidget.setDouble(_driveSS.rearLeftMotor.getMotorTemperature());
        backRightTempWidget.setDouble(_driveSS.rearRightMotor.getMotorTemperature());
        // Update current widgets
        frontLeftCurrentWidget.setDouble(_driveSS.frontLeftMotor.getOutputCurrent());
        frontRightCurrentWidget.setDouble(_driveSS.frontRightMotor.getOutputCurrent());
        backLeftCurrentWidget.setDouble(_driveSS.rearLeftMotor.getOutputCurrent());
        backRightCurrentWidget.setDouble(_driveSS.rearRightMotor.getOutputCurrent());
        // Update encoder widgets
        leftEncoderWidget.setDouble(_driveSS.leftEncoder.getPosition());
        rightEncoderWidget.setDouble(_driveSS.rightEncoder.getPosition());
        // Check for and handle change in drive mode selection:
        Command c = driveModeChooser.getSelected();
        if (c != manualC) {
            manualC = c;
            commandChange = true;
        }

        // LL-AutoAim tab:
        llaWidget.setBoolean(_rc.manualDriveArcade.llaaActive);
        lltxWidget.setDouble(_llSS.tx);
        lltoWidget.setDouble(_rc.manualDriveArcade.rootTurn);
        lltvWidget.setBoolean(_llSS.targetFound);
        if (llpWidget.getDouble(0) != LLP || lldWidget.getDouble(0) != LLD) {
            _rc.manualDriveArcade.pid = new PIDController(llpWidget.getDouble(0), Constants.pidConstants.LLI, lldWidget.getDouble(0));
            _rc.manualDriveArcade.pid.reset();
        }
    }
}
