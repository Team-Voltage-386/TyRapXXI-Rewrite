// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pidConstants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;

/**ArcadeDrive teleop command with button to enable LL-AutoAim*/
public class C_ManualDriveArcade extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem _dss;
  private final LLSubsystem _lls;
  private final PIDController pid = new PIDController(pidConstants.LLP, pidConstants.LLI, pidConstants.LLD);
  private final int _aimButton;
  public Boolean llaaActive = false;

  /**ArcadeDrive teleop command with button to enable LL-AutoAim
   * @param DSS The drive subsystem used by this command.
   * @param LLS the LL subsystem used by this command
   * @param aimButton the controller button used to activate LL-AutoAim
   */
  public C_ManualDriveArcade(DriveSubsystem DSS, LLSubsystem LLS, int aimButton) {
    _dss = DSS;
    _lls = LLS;
    _aimButton = aimButton;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
    addRequirements(_lls);

    pid.setTolerance(3,5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lls.driverMode(true);
    rootForward = 0;
    rootTurn = 0;
  }

  public double rootForward, rootTurn;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rootForward = RobotContainer.driverController.getRawAxis(kLeftVertical);

    // if rbumper and target found get and not met target set turn from pid
    if (RobotContainer.driverController.getRawButton(_aimButton) && _lls.targetFound && !pid.atSetpoint()) {
      llaaActive = true;
      if (RobotContainer.driverController.getRawButtonPressed(_aimButton)) {
        pid.reset(); // if it's the first iteration in this loop reset the pid and disable drivermode
        _lls.driverMode(false);
      }
      rootTurn = MathUtil.clamp(-1*pid.calculate(_lls.tx, 0), -1*pidConstants.LLC, pidConstants.LLC) ; // clamps the output of the PID to prevent murder bot
    }
    else {
      llaaActive = false;
      rootTurn = -1 * RobotContainer.driverController.getRawAxis(kRightHorizontal); // else get turn from remote
      if (RobotContainer.driverController.getRawButtonReleased(_aimButton)) _lls.driverMode(true); // if camera not switched back to driver mode do it
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
