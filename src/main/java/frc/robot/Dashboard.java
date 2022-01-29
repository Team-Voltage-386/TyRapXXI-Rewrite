package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.LLSubsystem;

/**Carl's wacky dash*/
public class Dashboard {
    // Creates a shuffleboard tab
    private static ShuffleboardTab llTab = Shuffleboard.getTab("LL-AutoAim");
    private static ShuffleboardTab llbTab = Shuffleboard.getTab("LL-ChaseBall");
    
    // Diagnostics in:
    private static LLSubsystem _llSS;
    private static RobotContainer _rc;
    private static NetworkTableEntry llaWidget = llTab.add("AutoAim Active",false).withSize(1,1).withPosition(0,0).getEntry();
    private static NetworkTableEntry lltxWidget = llTab.add("Process Variable Error",0).withSize(2,1).withPosition(0,1).getEntry();
    private static NetworkTableEntry lltoWidget = llTab.add("PID Output",0).withSize(1,1).withPosition(0,2).getEntry();
    private static NetworkTableEntry lltvWidget = llTab.add("Target Found",false).withSize(1,1).withPosition(1, 0).getEntry();
    private static NetworkTableEntry lltdWidget = llTab.add("Target Distance",0).withSize(1,1).withPosition(1,2).getEntry();

    private static LLSubsystem _llSSB;
    private static NetworkTableEntry llbaWidget = llbTab.add("AutoAim Active",false).withSize(1,1).withPosition(0,0).getEntry();
    private static NetworkTableEntry llbtxWidget = llbTab.add("Process Variable Error",0).withSize(2,1).withPosition(0,1).getEntry();
    private static NetworkTableEntry llbtoWidget = llbTab.add("PID Output",0).withSize(1,1).withPosition(0,2).getEntry();
    private static NetworkTableEntry llbtvWidget = llbTab.add("Target Found",false).withSize(1,1).withPosition(1, 0).getEntry();
    private static NetworkTableEntry llbtdWidget = llbTab.add("Target Distance",0).withSize(1,1).withPosition(1,2).getEntry();

    /**
     *  Initialize the dash, customize at will
     */
    public static void init() {
        _rc = Robot.m_robotContainer;
        _llSS = _rc.limeLightSubsystemHoop;
        _llSSB = _rc.limeLightSubsystemBall;
    }

    /**
     * Update the dash, customize at will
     */
    public static void update() {
        // LL-AutoAim tab:
        llaWidget.setBoolean(_rc.teleOpCommand.llaa);
        lltxWidget.setDouble(_llSS.tx);
        lltoWidget.setDouble(_rc.teleOpCommand.rootTurn);
        lltvWidget.setBoolean(_llSS.targetFound);
        lltdWidget.setDouble(_llSS.metersToTarget());

        llbaWidget.setBoolean(_rc.teleOpCommand.llcb);
        llbtxWidget.setDouble(_llSSB.tx);
        llbtoWidget.setDouble(_rc.teleOpCommand.rootTurn);
        llbtvWidget.setBoolean(_llSSB.targetFound);
        llbtdWidget.setDouble(_llSSB.metersToTarget());
    }
}