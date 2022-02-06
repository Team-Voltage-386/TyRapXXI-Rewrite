// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LLSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.pidConstants;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;

/**Driver TeleOp Command*/
public class D_TeleOp extends CommandBase {
  private final DriveSubsystem _dss;
  public final LLSubsystem _llss;
  private final LLSubsystem _llssb;
  private final Joystick _controller;
  private final PIDController pid = new PIDController(pidConstants.TP, pidConstants.TI, pidConstants.TD);
  private Boolean llaa = false;
  private Boolean llcb = false;
  private double rootForward, rootTurn;
  public Boolean ballFound = false;
  public Boolean hoopTargeted = false;
  public Boolean hoopLocked = false;

  private final ShuffleboardTab _tab;
  private final NetworkTableEntry hfWidget;
  private final NetworkTableEntry bfWidget;
  private final NetworkTableEntry hdWidget;
  private final SendableChooser<Integer> ballColorSelect = new SendableChooser<Integer>();

  /**Driver TeleOp Command
   * @param DSS The drive subsystem used by this command.
   * @param LLS the hoop LL subsystem used by this command.
   * @param LLSB the ball LL subsystem used by this command.
   */
  public D_TeleOp(DriveSubsystem DSS, LLSubsystem LLS, LLSubsystem LLSB, ShuffleboardTab t) {
    _tab = t;
    hdWidget = _tab.add("HoopDistance",0).withSize(1,1).withPosition(4,3).getEntry();
    hfWidget = _tab.add("HoopFound",false).withSize(1,1).withPosition(0,2).getEntry();
    bfWidget = _tab.add("BallFound",false).withSize(1,1).withPosition(1,2).getEntry();
    _dss = DSS;
    _llss = LLS;
    _llssb = LLSB;
    _controller = RobotContainer.driverController;
    _llss.driverMode(false);
    _llssb.driverMode(false);
    ballColorSelect.setDefaultOption("Yellow",2);
    ballColorSelect.addOption("Blue",1);
    ballColorSelect.addOption("Red",0);
    _tab.add(ballColorSelect).withSize(1,1).withPosition(1,1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
    addRequirements(_llss);
    addRequirements(_llssb);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {
    _llss.driverMode(false);
    rootForward = 0;
    rootTurn = 0;
    pid.setTolerance(1,1);
    _llss.targetLostWait = true;
    _llssb.setPipeLine(ballColorSelect.getSelected());
  }

  /**Called every time the scheduler runs while the command is scheduled.*/
  @Override
  public void execute() {
    rootForward = RobotContainer.driverController.getRawAxis(kLeftVertical);

    Boolean hl = false;
    Boolean ht = false;
    Boolean bf = false;
    float rumble = 0;

    if (_controller.getRawButton(kLeftBumper)) {llaa = true; llcb = false;} // if a is pressed, activate LLAA
    else if (_controller.getRawButton(kRightJoystickPressed)) {llcb = true; llaa = false;}
    else {llaa = false; llcb = false;}
    if (llaa && _llss.targetFound) {
      ht = true;
      if ((-1 < _llss.tx && _llss.tx < 1)) {
        hl = true;
        rootTurn = 0;
        rumble = (float)0.5;
      } else {
        hl = false;
        if (_controller.getRawButtonPressed(kLeftBumper)) pid.reset();
        if (_llss.targetFound) rootTurn = MathUtil.clamp(pid.calculate(_llss.tx, 0), -1*pidConstants.TC, pidConstants.TC); // clamps the pid output to prevent murderbot
      }
    } else if (llcb && _llssb.targetFound) {
      bf = true;
      if (_controller.getRawButtonPressed(kRightJoystickPressed)) pid.reset();
      rootTurn = MathUtil.clamp(pid.calculate(_llssb.tx, 0), -1*pidConstants.TC, pidConstants.TC);
    } else {
      rootTurn = -1 * RobotContainer.driverController.getRawAxis(kRightHorizontal); // else get turn from remote
    }

    hoopTargeted = ht;
    hoopLocked = hl;
    ballFound = bf;
    _controller.setRumble(RumbleType.kRightRumble, rumble);

    _dss.arcadeDrive(rootForward, rootTurn);
    updateWidgets();
  }

  private void updateWidgets() {
    hdWidget.setDouble(_llss.metersToTarget());
    hfWidget.setBoolean(_llss.targetFound);
    bfWidget.setBoolean(_llssb.targetFound);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}