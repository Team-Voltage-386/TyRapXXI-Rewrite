package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;

/**Carl's wacky dash*/
public class Dashboard {
    // Creates a shuffleboard tab
    private static ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Dashboard");
    private static ShuffleboardTab llTab = Shuffleboard.getTab("LL-AutoAim");
    
    // Diagnostics in:
    private static LLSubsystem _llSS;
    private static RobotContainer _rc;
    private static NetworkTableEntry llaWidget = llTab.add("AutoAim Active",false).withSize(1,1).withPosition(0,0).getEntry();
    private static NetworkTableEntry lltxWidget = llTab.add("Process Variable Error",0).withSize(2,1).withPosition(0,1).getEntry();
    private static NetworkTableEntry lltoWidget = llTab.add("PID Output",0).withSize(2,1).withPosition(0,2).getEntry();
    private static NetworkTableEntry lltvWidget = llTab.add("Target Found",false).withSize(1,1).withPosition(1, 0).getEntry();
    private static NetworkTableEntry lltdWidget = llTab.add("Target Distance",0).withSize(1,1).withPosition(4,0).getEntry();
    public static SendableChooser<Boolean> lltlwSelect = new SendableChooser<Boolean>();

    /**
     *  Initialize the dash, customize at will
     */
    public static void init() {
        _rc = Robot.m_robotContainer;
        _llSS = _rc.limeLightSubsystemHoop;
        //Init LLTLW select:
        lltlwSelect.setDefaultOption("False", false);
        lltlwSelect.addOption("True", true);
        llTab.add("LLTLW",lltlwSelect);
    }

    /**
     * Update the dash, customize at will
     */
    public static void update() {
        // LL-AutoAim tab:
        llaWidget.setBoolean(_rc.manualDriveArcade.llaaActive);
        lltxWidget.setDouble(_llSS.tx);
        lltoWidget.setDouble(_rc.manualDriveArcade.rootTurn);
        lltvWidget.setBoolean(_llSS.targetFound);
        lltdWidget.setDouble(_llSS.metersToTarget());
    }
}
