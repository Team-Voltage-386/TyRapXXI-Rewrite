// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HoodSubsystem;
import static frc.robot.Constants.ControllerConstants.*;

import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ManualHoodCommand extends CommandBase {
  private final HoodSubsystem m_subsystem;

  /** Creates a new ManualHoodCommand. */
  public ManualHoodCommand(HoodSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runHood(RobotContainer.manipulatorController.getRawAxis(kRightVertical));
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
