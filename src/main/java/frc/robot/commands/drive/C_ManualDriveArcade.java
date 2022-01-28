// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pidConstants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;

/**ArcadeDrive teleop command with button to enable LL-AutoAim*/
public class C_ManualDriveArcade extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem _dss;
  private final LLSubsystem _lls;
  private final Joystick _controller;
  public PIDController pid = new PIDController(_p, pidConstants.LLI, _d);
  private final double _seekTurnSpeed;
  public double _p = pidConstants.LLP;
  public double _d = pidConstants.LLD;
  public Boolean llaaActive = false;

  /**ArcadeDrive teleop command with button to enable LL-AutoAim
   * @param DSS The drive subsystem used by this command.
   * @param LLS the LL subsystem used by this command
   * @param seekTurnSpeed The speed at which to seek for a target
   */
  public C_ManualDriveArcade(DriveSubsystem DSS, LLSubsystem LLS, double seekTurnSpeed) {
    _dss = DSS;
    _lls = LLS;
    _seekTurnSpeed = seekTurnSpeed;
    _controller = RobotContainer.driverController;
    _lls.driverMode(false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
    addRequirements(_lls);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //_lls.driverMode(true);
    rootForward = 0;
    rootTurn = 0;
    pid.setTolerance(4,10);
  }

  public double rootForward, rootTurn;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rootForward = RobotContainer.driverController.getRawAxis(kLeftVertical);

    if (_controller.getRawButton(kRightBumper) || _controller.getRawButton(kLeftBumper)) llaaActive = true; // if a bumper is pressed, activate LLAA
    else llaaActive = false;

    if (llaaActive) {
      /*if (_controller.getRawButtonPressed(kRightBumper) || _controller.getRawButtonPressed(kLeftBumper)) { 
        pid.reset();
      }*/
      if (_lls.targetFound) rootTurn = MathUtil.clamp(pid.calculate(_lls.tx, 0), -1*pidConstants.LLC, pidConstants.LLC);
      else if (_controller.getRawButton(kRightBumper)) rootTurn = -1*_seekTurnSpeed;
      else if (_controller.getRawButton(kLeftBumper)) rootTurn = _seekTurnSpeed;
    } else {
      rootTurn = -1 * RobotContainer.driverController.getRawAxis(kRightHorizontal); // else get turn from remote
      if (_controller.getRawButtonReleased(kRightBumper) || _controller.getRawButtonReleased(kLeftBumper)) {
        _controller.setRumble(GenericHID.RumbleType.kRightRumble, 0); // stop the rumble
      }
    }
    _dss.arcadeDrive(rootForward, rootTurn);
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
