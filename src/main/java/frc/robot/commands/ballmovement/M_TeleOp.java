package frc.robot.commands.ballmovement;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;
import frc.robot.commands.drive.D_TeleOp;

/**Manipulator TeleOp Command*/
public class M_TeleOp extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final Joystick _controller;
    private Boolean intakeDeployed = false;
    private final D_TeleOp driver;
    private final ShuffleboardTab _tab;
    private final NetworkTableEntry rtfWidget;
    

    /**Manipulator TeleOp Command*/
    public M_TeleOp(BallMovementSubsystem BMSS, ShuffleboardTab t,D_TeleOp d) {
        _tab = t;
        rtfWidget = _tab.add("Ready To Fire",false).withSize(1,2).withPosition(3,0).getEntry();
        _bmss = BMSS;
        _bmss.reCalibrate();
        _controller = RobotContainer.manipulatorController;
        addRequirements(_bmss);
        _bmss.stop();
        driver = d;
    }

    @Override
    public void initialize() {
        _bmss.reCalibrate();
    }

    @Override
    public void execute() {
        if(_controller.getRawButtonPressed(kY)) {
            intakeDeployed = !intakeDeployed;
            _bmss.deployIntake(intakeDeployed);
        }

        if (_controller.getRawButtonPressed(kX)) _bmss.autoSF = !_bmss.autoSF;
        if (_controller.getRawButtonPressed(kB)) _bmss.drumIdle = !_bmss.drumIdle;

        if (_controller.getRawButton(kLeftBumper)) _bmss.runIntake(true);
        else _bmss.runIntake(false);
        if (driver.hoopTargeted) {
            _bmss.setAimDistance(driver._llss.metersToTarget());
            _bmss.drumControllerOn = true;
            if (driver.hoopLocked && _bmss.RTF()){
                if(_controller.getRawButton(kA)) _bmss.runFeeder(true);
                else {
                    _bmss.runFeeder(false);
                    _controller.setRumble(RumbleType.kRightRumble,0.5);
                    rtfWidget.setBoolean(true);
                }
            } else {
                _bmss.runFeeder(false);
                _controller.setRumble(RumbleType.kRightRumble,0);
                rtfWidget.setBoolean(false);
            }
        } else {
            _bmss.drumControllerOn = false;
            _bmss.runFeeder(false);
            _controller.setRumble(RumbleType.kRightRumble,0);
            rtfWidget.setBoolean(false);
        }
    }

    @Override
    public void end(boolean interuppted) {
        _bmss.stop();
    }
}