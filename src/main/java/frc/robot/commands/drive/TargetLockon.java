// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import static frc.robot.Constants.ControllerConstants.*;

public class TargetLockon extends CommandBase {
  private final DriveSubsystem m_DriveSubsystem;
  private final LimelightSubsystem m_LimelightSubsystem;
  /** Creates a new TargetLockon. */
  public TargetLockon(DriveSubsystem subsystem1, LimelightSubsystem subsystem2) {
    m_DriveSubsystem=subsystem1;
    m_LimelightSubsystem=subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1);
    addRequirements(subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private final double kP = -.01;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.arcadeDrive(0.0, kP*m_LimelightSubsystem.getTargetX());
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
