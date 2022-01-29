// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.HoodConstants.*;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;

public class HoodSubsystem extends SubsystemBase {

  private ShuffleboardTab tab = Shuffleboard.getTab("Hood");

  // Create hood widgets
  private NetworkTableEntry hoodMotorOutputWidget = tab.add("Hood Motor Output", 0).withSize(2, 1).getEntry();
  private NetworkTableEntry hoodEncoderWidget = tab.add("Hood Encoder", 0).withSize(2, 1).getEntry();
  private NetworkTableEntry hoodBottomLimitWidget = tab.add("Hood Limit", false).withSize(2, 1).getEntry();

  // initialize motors and sensors
  public final TalonSRX hoodMotor = new TalonSRX(kHoodMotor);
  public final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoder);
  public final DigitalInput hoodLimit = new DigitalInput(kHoodLimit);

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    hoodMotor.configFactoryDefault();
    hoodMotor.configNeutralDeadband(0.1);
    didReset = false;
  }

  // Constants
  private final double kMaxHoodEncoder = 0.6;

  // beginning boolean for resetting stuff
  private boolean didReset;

  /** @return true = hood at bottom position */
  public boolean getHoodBottomLimit() {
    return !hoodLimit.get();
  }

  public double getHoodEncoder() {
    return hoodEncoder.get();
  }

  /** @param power positive = hood down (increases launch angle) */
  public void runHood(double power) {
    boolean noGo = ((power > 0.0 && getHoodBottomLimit()) || (power < 0.0 && getHoodEncoder() >= kMaxHoodEncoder));
    if (!noGo) {
      hoodMotor.set(ControlMode.PercentOutput, power);
    } else {
      hoodMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    while (!didReset) {
      didReset = getHoodBottomLimit();
      if (!didReset) {
        runHood(0.1);
      }
    }
    if (getHoodBottomLimit()) {
      hoodEncoder.reset();
    }

    // Update hood widgets
    hoodMotorOutputWidget.setDouble(hoodMotor.getMotorOutputPercent());
    hoodEncoderWidget.setDouble(getHoodEncoder());
    hoodBottomLimitWidget.setBoolean(getHoodBottomLimit());
  }
}
