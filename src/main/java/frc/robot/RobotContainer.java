// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // drivecontroller
  public static final Joystick driverController = new Joystick(0);
  public static final JoystickButton targetingButton = new JoystickButton(driverController, 6);

  // manipulatorcontroller
  public static final Joystick manipulatorController = new Joystick(1);

  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final LLSubsystem limeLightSubsystem = new LLSubsystem();
  public final ManualDriveTank manualDriveTank = new ManualDriveTank(driveSubsystem);
  public final C_ManualDriveArcade manualDriveArcade = new C_ManualDriveArcade(driveSubsystem, limeLightSubsystem, 0.6);
  public Command manualCommand;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // configure default commands

    driveSubsystem.setDefaultCommand(manualDriveArcade);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public Command getTeleOpCommand() {
    return Dashboard.manualC;
  }
}
