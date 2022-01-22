// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/* Add your docs here. */
public class LimelightSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry targetvalid = table.getEntry("tv");//valid target 0/1
  private NetworkTableEntry targetx = table.getEntry("tx");//target x offset degrees
  private NetworkTableEntry targety = table.getEntry("ty");//target y offset degrees
  private NetworkTableEntry targetarea = table.getEntry("ta");//target area 0-100%

  // create shuffleboard tab for limelight
  private ShuffleboardTab tab = Shuffleboard.getTab("limelight");
  //shuffleboard widgets
  private NetworkTableEntry txWidget = tab.add("tx", 2048).withPosition(0, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry tyWidget = tab.add("ty", 2048).withPosition(2, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry taWidget = tab.add("ta", 2048).withPosition(4, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry tvWidget = tab.add("tv", false).withPosition(6, 0).withSize(2, 1).getEntry();


  public void periodic() {
    // read values periodically
    boolean valid = targetvalid.getBoolean(false);
    double x = targetx.getDouble(0.0);
    double y = targety.getDouble(0.0);
    double area = targetarea.getDouble(0.0);

    // post to shuffleboard
    txWidget.setDouble(x);
    tyWidget.setDouble(y);
    taWidget.setDouble(area);
    tvWidget.setBoolean(valid);
  }

}
