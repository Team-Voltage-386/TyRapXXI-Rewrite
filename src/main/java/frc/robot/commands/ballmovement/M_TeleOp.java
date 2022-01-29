package frc.robot.commands.ballmovement;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;

/**Manipulator TeleOp Command*/
public class M_TeleOp extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final Joystick _controller;
    private Boolean intakeDeployed = false;
    private Boolean feederRunning = false;
    private Boolean intakeRunning = false;
    private Boolean serializerRunning = false;
    //private int ballCount;

    /**Manipulator TeleOp Command*/
    public M_TeleOp(BallMovementSubsystem BMSS) {
        _bmss = BMSS;
        _controller = RobotContainer.manipulatorController;
        addRequirements(_bmss);
        stop();
    }

    @Override
    public void execute() {
        if (_controller.getRawButtonPressed(ControllerConstants.kY)) {
            intakeDeployed = !intakeDeployed;
            _bmss.deployIntake(intakeDeployed);
        }
        if (_controller.getRawButtonPressed(ControllerConstants.kX)) {
            serializerRunning = !serializerRunning;
            _bmss.runSerializer(serializerRunning);
        }
        if (_controller.getRawButtonPressed(ControllerConstants.kB)) {
            intakeRunning = !intakeRunning;
            _bmss.runIntake(intakeRunning);
        }
        if (_controller.getRawButtonPressed(ControllerConstants.kA)) {
            feederRunning = !feederRunning;
            _bmss.runFeeder(feederRunning);
        }
        _bmss.setLauncherPower(_controller.getRawAxis(ControllerConstants.kRightTrigger));
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