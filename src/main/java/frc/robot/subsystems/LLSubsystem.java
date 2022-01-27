package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**Carl's attempt at making a lime-light subsystem*/
public class LLSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public NetworkTable _nt;
  public Boolean targetFound;
  public float tx;
  public float ty;
  public float ta;
  public float ts;

  /**Carl's attempt at making a lime-light subsystem*/
  public LLSubsystem() {
      _nt = NetworkTableInstance.getDefault().getTable("limelight");
      driverMode(true);
  }

  /**During periodic this subsystem updates the basic public LL variables*/
  @Override
  public void periodic() {
      if(_nt.getEntry("tv").getDouble(-1) == 0) targetFound = false;
      else {
          targetFound = true;
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
      if (b) _nt.getEntry("camMode").setNumber(1);
      else _nt.getEntry("camMode").setNumber(0);
  }
}