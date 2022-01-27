// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/* Add your docs here. */
public class LimelightSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = table.getEntry("tv");// valid target 0/1
  private NetworkTableEntry tx = table.getEntry("tx");// target x offset degrees
  private NetworkTableEntry ty = table.getEntry("ty");// target y offset degrees
  private NetworkTableEntry ta = table.getEntry("ta");// target area 0-100%

  // create shuffleboard tab for limelight
  private ShuffleboardTab tab = Shuffleboard.getTab("limelight");
  // shuffleboard widgets
  private NetworkTableEntry txWidget = tab.add("tx", 2048).withPosition(0, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry tyWidget = tab.add("ty", 2048).withPosition(2, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry taWidget = tab.add("ta", 2048).withPosition(4, 0).withSize(2, 1).getEntry();
  private NetworkTableEntry tvWidget = tab.add("target valid", false).withPosition(6, 0).withSize(2, 1).getEntry();

  private double targetValid, targetX, targetY, targetArea;

  public void periodic() {
    // read values periodically
    targetValid = tv.getDouble(0);
    targetX = tx.getDouble(0.0);
    targetY = ty.getDouble(0.0);
    targetArea = ta.getDouble(0.0);

    // post to shuffleboard
    txWidget.setDouble(targetX);
    tyWidget.setDouble(targetY);
    taWidget.setDouble(targetArea);
    tvWidget.setBoolean((targetValid == 1));
  }

  public double getTargetX() {
    return targetX;
  }

  public double getTargetY() {
    return targetY;
  }

  public boolean getTargetValid() {
    return (targetValid == 1);
  }

}
