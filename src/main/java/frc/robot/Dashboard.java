package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.BallMovementSubsystem;

/**Carl's wacky dash*/
public class Dashboard {
    // Creates a shuffleboard tab
    private static ShuffleboardTab bmTab = Shuffleboard.getTab("Ball-Movement");
    
    // Diagnostics in:
    private static RobotContainer _rc;
    private static BallMovementSubsystem _bmSS;
    private static NetworkTableEntry hPosition = bmTab.add("Hood-Pos.",0).withSize(1,1).withPosition(1,2).getEntry();
    private static NetworkTableEntry hLL = bmTab.add("HLL",false).withSize(1,1).withPosition(2,0).getEntry();
    private static NetworkTableEntry hsp = bmTab.add("HSP",0).withSize(1,1).withPosition(2, 1).getEntry();
    private static NetworkTableEntry lv = bmTab.add("LV",0).withSize(1,1).withPosition(2, 2).getEntry();
    private static NetworkTableEntry tD = bmTab.add("TD",0).withSize(1,1).withPosition(0,0).getEntry();

    /**
     *  Initialize the dash, customize at will
     */
    public static void init() {
        _rc = Robot.m_robotContainer;
        _bmSS = _rc.ballMovementSS;
    }

    /**
     * Update the dash, customize at will
     */
    public static void update() {
        // BallMovement tab:
        hPosition.setDouble(_bmSS.hoodPosition);
        hLL.setBoolean(_rc.ballMovementSS.hoodLowLimit);
        hsp.setDouble(_rc.teleOpM.hoodPosition);
        lv.setDouble(_rc.ballMovementSS.launcherCurrentSpeed);
        tD.setDouble(_rc.limeLightSubsystemHoop.metersToTarget());
    }
}