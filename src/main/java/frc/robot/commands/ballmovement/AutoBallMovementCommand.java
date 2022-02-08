// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BallMovementSubsystem;

import static frc.robot.Constants.ControllerConstants.*;

public class AutoBallMovementCommand extends CommandBase {
  BallMovementSubsystem subsystem;
  Timer overallMotorTimer = new Timer();
  Timer motorTimer = new Timer();
  protected boolean intakeLatch = false;// intake deployed status

  /** Creates a new ManualBallMovementCommand. */
  public AutoBallMovementCommand(BallMovementSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    // Set initial state of intake as retracted
    subsystem.retractIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* intake deploy latch switch mode */
    if (RobotContainer.manipulatorController.getRawButtonPressed(kA)) {
      intakeLatch = !intakeLatch;
    }

    if (intakeLatch) {
      subsystem.deployIntake();
    } else {
      subsystem.retractIntake();
    }

    // Manually run motors with joysticks
    subsystem.runFeeder(RobotContainer.manipulatorController.getRawAxis(kRightVertical));
    subsystem.runLauncher(RobotContainer.manipulatorController.getRawAxis(kLeftTrigger));
    subsystem.runSerializerMotor(RobotContainer.manipulatorController.getRawAxis(kLeftVertical));
    subsystem.runIntakeMotor(RobotContainer.manipulatorController.getRawAxis(kRightTrigger));
    boolean launch = false;
    if (RobotContainer.manipulatorController.getRawButtonPressed(kB) == true) {
      launch = true;
      overallMotorTimer.start(); 
      if (overallMotorTimer.get() <= 5) { // dont actually know what the set time set is, assuming 5 seconds for ability to test code 
        // run feeder motor and serializer motor for set time to shoot 3 balls
        // the run function for the motor's parameter is power
        // power is from -1.0 (for reverse) to 1.0 (for forward)
      } else {
        // stop running feeder and serializer 
      }
    } else if (launch == true) {
      if (overallMotorTimer.get() <= 5) {
        // continue running feeder and serializer motor for the set amount of time
      }
    } else {
      if (subsystem.getIndexerSensor() == false && subsystem.getFeederSensor() == true) {
        // make serializer and feeder (motor) run
      } else if (subsystem.getEntranceSensor() == false && subsystem.getIndexerSensor() == false
          && subsystem.getFeederSensor() == false) {
        // do nothing (new)
      } else if (subsystem.getEntranceSensor() == true && subsystem.getIndexerSensor() == true
          && subsystem.getFeederSensor() == true) {
        motorTimer.start();
        if (motorTimer.get() <= 1) {
          // run serializer for short period of time to have all balls in serializer
          // dont actually know how long  it should take, assuming 1 second for ability to test code
        }
      } else if (subsystem.getEntranceSensor() == true) {
        // make serializer run
      } else { // entrance senor is false
        // make serializer false
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
