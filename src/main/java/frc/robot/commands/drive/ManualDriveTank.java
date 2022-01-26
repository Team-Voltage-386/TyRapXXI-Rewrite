// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

import static frc.robot.Constants.ControllerConstants.*;

/** An example command that uses an example subsystem. */
public class ManualDriveTank extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ManualDriveTank(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rootLeft = 0;
        rootRight = 0;
    }

    private double rootLeft, rootRight;

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rootLeft = RobotContainer.driverController.getRawAxis(kLeftVertical);
        rootRight = RobotContainer.driverController.getRawAxis(kRightVertical);
        m_subsystem.tankDrive(rootLeft, rootRight);
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
