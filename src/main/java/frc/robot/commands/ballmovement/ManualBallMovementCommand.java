// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BallMovementSubsystem;

import static frc.robot.Constants.ControllerConstants.*;

public class ManualBallMovementCommand extends CommandBase {

  BallMovementSubsystem subsystem;

  protected boolean intakeLatch = false;// intake deployed status

  /** Creates a new ManualBallMovementCommand. */
  public ManualBallMovementCommand(BallMovementSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
