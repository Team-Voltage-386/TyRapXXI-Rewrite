package frc.robot.commands.ballmovement;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;
import frc.robot.Constants.BallMovementConstants;
import frc.robot.commands.drive.D_TeleOp;

/**Manipulator TeleOp Command*/
public class M_TeleOp extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final Joystick _controller;
    private Boolean intakeDeployed = false;
    private double hoodPosition = 0;
    private final D_TeleOp driver;
    private final Timer ballCatchTimer = new Timer();
    private final ShuffleboardTab _tab;
    private final NetworkTableEntry rtfWidget;
    private Boolean serializer = false;
    

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
        ballCatchTimer.reset();
        ballCatchTimer.start();
    }

    @Override
    public void execute() {

        if(_controller.getRawButtonPressed(kY)) {
            intakeDeployed = !intakeDeployed;
            _bmss.deployIntake(intakeDeployed);
        }

        double hs = _controller.getRawAxis(kRightVertical);
        if (hs > 0.1 || hs < -0.1) {
            hoodPosition += hs*BallMovementConstants.manHoodSpeed;
            _bmss.hoodSet = hoodPosition;
        }

        if (_controller.getRawButtonPressed(kX)) {
            serializer = !serializer;
            _bmss.runSerializer(serializer);
        }

        if (!ballCatchTimer.hasElapsed(1) || _controller.getRawButton(kLeftBumper)) _bmss.runIntake(true);
        else _bmss.runIntake(false);
        if (driver.hoopTargeted) {
            _bmss.launcherControllerOn = true;
            if (driver.hoopLocked && _bmss.launcherAtSpeed()){
                if(_controller.getRawButton(kA)) _bmss.runFeeder(true);
                else {
                    _bmss.runFeeder(false);
                    _controller.setRumble(RumbleType.kRightRumble,0.6);
                    rtfWidget.setBoolean(true);
                }
            } else {
                _bmss.runFeeder(false);
                _controller.setRumble(RumbleType.kRightRumble,0);
                rtfWidget.setBoolean(false);
            }
        } else {
            _bmss.launcherControllerOn = false;
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