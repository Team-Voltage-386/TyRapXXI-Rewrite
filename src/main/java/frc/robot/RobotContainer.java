// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.ballmovement.DirectInputFire;
import frc.robot.commands.ballmovement.M_TeleOp;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.LimeLightConstants;

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

  // The robot's subsystems and commands are defined here...
  public final ShuffleboardTab sbTab = Shuffleboard.getTab("The One Tab to Rule Them All");
  public final ShuffleboardTab aTab = Shuffleboard.getTab("AutoTab");
  public final DriveSubsystem driveSubsystem;
  public final BallMovementSubsystem ballMovementSS = new BallMovementSubsystem(sbTab);
  public final LLSubsystem limeLightSubsystemHoop = new LLSubsystem("limelight",LimeLightConstants.targetHeightHoop,LimeLightConstants.camEleAngleHoop,LimeLightConstants.camHeightHoop);
  public final LLSubsystem limeLightSubsystemBall = new LLSubsystem("limelight-ball",LimeLightConstants.targetHeightBall,LimeLightConstants.camEleAngleBall,LimeLightConstants.camHeightBall);
  public final D_TeleOp teleOpD;
  public final LinearDrive autoCommand;
  public final M_TeleOp teleOpM;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    driveSubsystem = new DriveSubsystem(aTab);
    teleOpD = new D_TeleOp(driveSubsystem, limeLightSubsystemHoop, limeLightSubsystemBall, sbTab);
    teleOpM = new M_TeleOp(ballMovementSS, sbTab, teleOpD);
    autoCommand = new LinearDrive(driveSubsystem, aTab, 5);
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(teleOpD);
    ballMovementSS.setDefaultCommand(teleOpM);
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
    return autoCommand;
  }

  public ParallelCommandGroup getTeleOpCommands() {
    return new ParallelCommandGroup(teleOpD,teleOpM);
  }
}
