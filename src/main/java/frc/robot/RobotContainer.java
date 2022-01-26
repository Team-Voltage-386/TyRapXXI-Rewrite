// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.drive.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  // manipulatorcontroller
  public static final Joystick manipulatorController = new Joystick(1);
  //Driver Input Tab
  private ShuffleboardTab driverInputTab;
  private SendableChooser<Command> driveModeChooser = new SendableChooser();

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubSystem = new DriveSubsystem();
  private final ManualDriveTank manualDriveTank = new ManualDriveTank(driveSubSystem);
  private final ManualDriveArcade manualDriveArcade = new ManualDriveArcade(driveSubSystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // // Setup the Shuffleboard Tab
    driverInputTab = Shuffleboard.getTab("Driver Tab");
    //Instantiate DriveModeChooser
    driveModeChooser.setDefaultOption("ArcadeDrive", manualDriveArcade);
    driveModeChooser.addOption("TankDrive", manualDriveTank);
    driverInputTab.add(driveModeChooser);

    // configure default commands
    driveSubSystem.setDefaultCommand(getManualDriveCommand());

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

  public Command getManualDriveCommand() {
    return driveModeChooser.getSelected();
  }
}
