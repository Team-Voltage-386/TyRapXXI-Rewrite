package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.LimeLightConstants;

/**Carl's attempt at making a lime-light subsystem*/
public class LLSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public NetworkTable _nt;
  /**Whether or not the LL should wait before declaring target loss to see if it will come back*/
  public Boolean targetLostWait = false;
  public Boolean dMode = false;
  public Boolean targetFound = false;
  public float tx;
  public float ty;
  public float ta;
  public float ts;
  private double _TH;
  private double _MA;
  private double _MH;
  private Timer timer = new Timer();

  /**Carl's attempt at making a lime-light subsystem*/
  public LLSubsystem(String hostName, double targetHeight, double mountAngle, double mountHeight) {
      _nt = NetworkTableInstance.getDefault().getTable(hostName);
      _TH = targetHeight;
      _MA = mountAngle;
      _MH = mountHeight;
      driverMode(false);
      timer.start();
  }

  /**During periodic this subsystem updates the basic public LL variables*/
  @Override
  public void periodic() {
    targetLostWait = Dashboard.lltlwSelect.getSelected();
    if(_nt.getEntry("tv").getDouble(-1) == 0) {
      if(targetLostWait) targetFound = !timer.hasElapsed(LimeLightConstants.targetLostWaitTime); // if it should wait for target re-acquire, then wait, else declare it lost
      else targetFound = false;
    }
    else {
        targetFound = true;
        timer.reset();
        timer.start();
        tx = (float)_nt.getEntry("tx").getDouble(0);
        ty = (float)_nt.getEntry("ty").getDouble(0);
        ta = (float)_nt.getEntry("ta").getDouble(0);
        ts = (float)_nt.getEntry("ts").getDouble(0);
    }
  }

  /**Empty*/
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Enable/Disable drivermode (Exposure turned up to make image visible to mere humans)
   * @param b whether or not drivermode should be turned on or off
   */
  public void driverMode(Boolean b) {
      dMode = b;
      if (b) _nt.getEntry("camMode").setNumber(1);
      else _nt.getEntry("camMode").setNumber(0);
  }

  /**Returns the meters to the target given a the target's height from the ground
  */
  public double metersToTarget() {
    return (_TH-_MH)/Math.tan(Math.PI*((_MA+ty)/180));
  }
}