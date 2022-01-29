package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;
import frc.robot.Constants.BallMovementConstants;

/**Manipulator TeleOp Command*/
public class M_TeleOp extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final Joystick _controller;
    private Boolean intakeDeployed = false;
    private Boolean feederRunning = false;
    private Boolean intakeRunning = false;
    private Boolean serializerRunning = false;
    private Boolean launcherRunning = false;
    public double hoodPosition = 0;
    //private int ballCount;

    /**Manipulator TeleOp Command*/
    public M_TeleOp(BallMovementSubsystem BMSS) {
        _bmss = BMSS;
        _bmss.reCalibrate();
        _controller = RobotContainer.manipulatorController;
        addRequirements(_bmss);
        stop();
    }

    @Override
    public void execute() {
        double hs = _controller.getRawAxis(kRightVertical);
        if (hs > 0.03 || hs < -0.03) {
            hoodPosition += hs*BallMovementConstants.manHoodSpeed;
            _bmss.setHoodPosition(hoodPosition);
        }
        if (_controller.getRawButton(kLeftBumper)) {
            Robot.m_robotContainer.limeLightSubsystemHoop.metersToTarget();
        }
        if (_controller.getRawButtonPressed(kY)) {
            intakeDeployed = !intakeDeployed;
            _bmss.deployIntake(intakeDeployed);
        }
        if (_controller.getRawButtonPressed(kX)) {
            serializerRunning = !serializerRunning;
            _bmss.runSerializer(serializerRunning);
        }
        if (_controller.getRawButtonPressed(kA)) {
            intakeRunning = !intakeRunning;
            _bmss.runIntake(intakeRunning);
        }
        if (_controller.getRawButtonPressed(kB)) {
            feederRunning = !feederRunning;
            _bmss.runFeeder(feederRunning);
        }
        if (_controller.getRawButtonPressed(kRightBumper)) {
            _bmss.pidL.reset();
            launcherRunning = !launcherRunning;
            _bmss.setLauncherOn(launcherRunning);
        }
    }

    @Override
    public void end(boolean interuppted) {
        stop();
    }

    public void stop() {
        _bmss.setLauncherPower(0);
        _bmss.deployIntake(false);
        _bmss.runFeeder(false);
        _bmss.runIntake(false);
        _bmss.runSerializer(false);
    }
}