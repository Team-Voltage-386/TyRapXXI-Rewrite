// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pidConstants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;

/**ArcadeDrive teleop command with bumpers to enable LL-AutoAim*/
public class C_TeleOp extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem _dss;
  private final LLSubsystem _llss;
  private final LLSubsystem _llssb;
  private final Joystick _controller;
  public PIDController pid = new PIDController(pidConstants.LLP, pidConstants.LLI, pidConstants.LLD);
  private final double _seekTurnSpeed;
  public Boolean llaa = false;
  public Boolean llcb = false;

  /**ArcadeDrive teleop command with button to enable LL-AutoAim
   * @param DSS The drive subsystem used by this command.
   * @param LLS the LL subsystem used by this command
   * @param seekTurnSpeed The speed at which to seek for a target
   */
  public C_TeleOp(DriveSubsystem DSS, LLSubsystem LLS, LLSubsystem LLSB, double seekTurnSpeed) {
    _dss = DSS;
    _llss = LLS;
    _llssb = LLSB;
    _seekTurnSpeed = seekTurnSpeed;
    _controller = RobotContainer.driverController;
    _llss.driverMode(false);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
    addRequirements(_llss);
    addRequirements(_llssb);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {
    _llss.driverMode(true);
    rootForward = 0;
    rootTurn = 0;
    pid.setTolerance(1,1);
    _llss.targetLostWait = true;
  }

  public double rootForward, rootTurn;

  /**Called every time the scheduler runs while the command is scheduled.*/
  @Override
  public void execute() {
    rootForward = RobotContainer.driverController.getRawAxis(kLeftVertical);

    if (_controller.getRawButton(kA)) {llaa = true; llcb = false; _llss.driverMode(false);} // if a is pressed, activate LLAA
    else if (_controller.getRawButton(kX)) {llcb = true; llaa = false; _llssb.driverMode(false);}
    else {llaa = false; llcb = false; _llss.driverMode(true); _llssb.driverMode(true);}

    if (llaa) {
      if ((-0.6 < _llss.tx && _llss.tx < 0.6) && _llss.targetFound) {
        rootTurn = 0;
        _controller.setRumble(RumbleType.kRightRumble, 0.5);
      } else {
        _controller.setRumble(RumbleType.kRightRumble,0);
        if (_controller.getRawButtonPressed(kA)) pid.reset();
        if (_llss.targetFound) rootTurn = MathUtil.clamp(pid.calculate(_llss.tx, 0), -1*pidConstants.LLC, pidConstants.LLC); // clamps the pid output to prevent murderbot
      }
    } else if (llcb && _llssb.targetFound) {
      if (_controller.getRawButtonPressed(kX)) pid.reset();
      rootTurn = MathUtil.clamp(pid.calculate(_llssb.tx, 0), -1*pidConstants.LLC, pidConstants.LLC);
    } else {
      _controller.setRumble(RumbleType.kRightRumble,0);
      rootTurn = -1 * RobotContainer.driverController.getRawAxis(kRightHorizontal); // else get turn from remote
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
