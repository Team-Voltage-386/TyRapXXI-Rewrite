package frc.robot.commands.ballmovement;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;
import frc.robot.commands.drive.D_TeleOp;

/**Manipulator TeleOp Command*/
public class M_TeleOp extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final Joystick _controller;
    private Boolean intakeDeployed = false;
    

    /**Manipulator TeleOp Command*/
    public M_TeleOp(BallMovementSubsystem BMSS) {
        _bmss = BMSS;
        _bmss.reCalibrate();
        _controller = RobotContainer.manipulatorController;
        addRequirements(_bmss);
        _bmss.stop();
    }

    @Override
    public void initialize() {
        _bmss.hoodSet = 0.61;
        _bmss.reCalibrate();
    }

    @Override
    public void execute() {
        if (_controller.getRawButtonPressed(kY)) {
            intakeDeployed = !intakeDeployed;
            _bmss.deployIntake(intakeDeployed);
        }

        _bmss.runIntake(_controller.getRawButton(kLeftBumper));
        if (_controller.getRawButtonPressed(kX)) _bmss.autoSF = !_bmss.autoSF;
        if (_controller.getRawButtonPressed(kB)) _bmss.drumIdle = !_bmss.drumIdle;
        if (Robot.m_robotContainer.hoopTargeted) {
            _bmss.setAimDistance(Robot.m_robotContainer.metersToTarget);
            _bmss.drumControllerOn = true;
            if (Robot.m_robotContainer.hoopLocked && _bmss.RTF()){
                if(_controller.getRawButton(kA)) _bmss.runFeeder(true);
                else {
                    _bmss.runFeeder(false);
                    _controller.setRumble(RumbleType.kRightRumble,0.5);
                }
            } else {
                _bmss.runFeeder(false);
                _controller.setRumble(RumbleType.kRightRumble,0);
            }
        } else {
            _bmss.drumControllerOn = false;
            _bmss.runFeeder(false);
            _controller.setRumble(RumbleType.kRightRumble,0);
        }
    }

    @Override
    public void end(boolean interuppted) {
        _bmss.stop();
    }
}