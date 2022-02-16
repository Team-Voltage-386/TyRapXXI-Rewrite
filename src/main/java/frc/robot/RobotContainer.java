// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ballmovement.M_TeleOp;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  public Boolean hoopTargeted = false;
  public Boolean hoopLocked = false;
  public double metersToTarget = 0;

  // The robot's subsystems and commands are defined here...
  public final BallMovementSubsystem ballMovementSS = new BallMovementSubsystem();
  public final LLSubsystem limeLightSubsystemHoop = new LLSubsystem("limelight-hoop",LimeLightConstants.targetHeightHoop,LimeLightConstants.camEleAngleHoop,LimeLightConstants.camHeightHoop);
  public final LLSubsystem limeLightSubsystemBall = new LLSubsystem("limelight-ball",LimeLightConstants.targetHeightBall,LimeLightConstants.camEleAngleBall,LimeLightConstants.camHeightBall);
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final D_TeleOp teleOpD = new D_TeleOp(driveSubsystem, limeLightSubsystemHoop, limeLightSubsystemBall);
  public final M_TeleOp teleOpM = new M_TeleOp(ballMovementSS);

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
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
    /*return new SequentialCommandGroup(  new LinearDrive(driveSubsystem, 1, 0, true),
                                        new StationaryTurn(driveSubsystem, -170, true),
                                        new StationaryTurn(driveSubsystem, -78, true),
                                        new LinearDrive(driveSubsystem, 2.65, 0, true),
                                        new StationaryTurn(driveSubsystem, -37, true),
                                        new LinearDrive(driveSubsystem, 3.5, 0, true),
                                        new StationaryTurn(driveSubsystem, 167.5, true));*/
      return new SequentialCommandGroup(  new LinearDrive(driveSubsystem,0.5,0,false),
                                          new StationaryTurn(driveSubsystem, 270, false),
                                          new LinearDrive(driveSubsystem,0.5,270,false),
                                          new StationaryTurn(driveSubsystem,0,false),
                                          new LinearDrive(driveSubsystem,0.5,0,false),
                                          new StationaryTurn(driveSubsystem, 270, false),
                                          new LinearDrive(driveSubsystem,0.5,270,false),
                                          new StationaryTurn(driveSubsystem,0,false),
                                          new LinearDrive(driveSubsystem,0.5,0,false));
  }

  public ParallelCommandGroup getTeleOpCommands() {
    return new ParallelCommandGroup(teleOpD,teleOpM);
  }
}
