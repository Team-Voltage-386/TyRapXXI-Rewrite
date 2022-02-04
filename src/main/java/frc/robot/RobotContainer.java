// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ballmovement.ManualBallMovementCommand;
import frc.robot.commands.drive.*;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants.*;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutonomousConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.List;

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
  private final DriveSubsystem driveSubSystem = new DriveSubsystem();
  private final BallMovementSubsystem ballMovementSubsystem = new BallMovementSubsystem();
  private final ManualBallMovementCommand ballMovementCommand = new ManualBallMovementCommand(ballMovementSubsystem);

  // Sendable chooser declarations
  // Shuffleboard declarations
  public static ShuffleboardTab driverTab;
  private SendableChooser<Boolean> teleopSendableChooser;
  private final ManualDriveTank manualDriveTankCommand = new ManualDriveTank(driveSubSystem);
  private final ManualDriveArcade manualDriveCommand = new ManualDriveArcade(driveSubSystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // // Instantiate Driver Tab
    // driverTab = Shuffleboard.getTab("Driver Tab");

    // configure default commands
    driveSubSystem.setDefaultCommand(manualDriveTankCommand);

    // configure default commands
    ballMovementSubsystem.setDefaultCommand(ballMovementCommand);
  }

  public Boolean getTeleopSendableChooser() {
    return teleopSendableChooser.getSelected();
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

    // create voltage constraint so we do not go too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(AutonomousConstants.ksVolts, AutonomousConstants.kvVoltSecondsPerMeter,
            AutonomousConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10.0);

    // create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutonomousConstants.kMaxSpeedMetersPerSecond,
        AutonomousConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // make trajectory
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(0, 3, new Rotation2d(0)), config);

    // make Ramsete Command
    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, driveSubSystem::getPose,
        new RamseteController(AutonomousConstants.kRamseteB, AutonomousConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutonomousConstants.ksVolts, AutonomousConstants.kvVoltSecondsPerMeter,
            AutonomousConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, driveSubSystem::getDifferentialDriveWheelSpeeds,
        new PIDController(AutonomousConstants.kPDriveVel, 0, 0),
        new PIDController(AutonomousConstants.kPDriveVel, 0, 0),
        driveSubSystem::tankDriveVolts, driveSubSystem);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubSystem.tankDriveVolts(0, 0));
  }

  public Command getManualDriveCommand() {
    return manualDriveTankCommand;
  }
}
