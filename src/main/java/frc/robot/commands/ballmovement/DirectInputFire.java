package frc.robot.commands.ballmovement;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMovementSubsystem;

/**Manipulator TeleOp Command*/
public class DirectInputFire extends CommandBase {

    private final BallMovementSubsystem _bmss;
    private final ShuffleboardTab _tab;
    private final NetworkTableEntry dsWidget;
    private final NetworkTableEntry hpWidget;
    private final NetworkTableEntry fsWidget;
    private final double hoodSet;
    private final int drumSet;
    private final Timer finTimer = new Timer();
    
    

    /**Manipulator TeleOp Command*/
    public DirectInputFire(BallMovementSubsystem BMSS, ShuffleboardTab t, int drumSpeed, double hoodPosition) {
        _tab = t;
        dsWidget = _tab.add("DrumSpeed",0).withSize(1,1).withPosition(0,1).getEntry();
        hpWidget = _tab.add("HoodPosition",0).withSize(1,1).withPosition(1,1).getEntry();
        fsWidget = _tab.add("feederSensor",false).withSize(1,1).withPosition(0, 2).getEntry();
        hoodSet = hoodPosition;
        drumSet = drumSpeed;
        _bmss = BMSS;
        addRequirements(_bmss);
    }

    @Override
    public void initialize() {
        finTimer.stop();
        finTimer.reset();
        _bmss.reCalibrate();
        _bmss.stop();
        _bmss.deployIntake(true);
        _bmss.setDrumPower(drumSet);
        _bmss.drumControllerOn = true;
        _bmss.hoodSet = hoodSet;
        _bmss.drumSP = drumSet;
    } 

    @Override
    public void execute() {
        updateWidgets();
        if (!_bmss.feed) {
            _bmss.runFeedSlow(true);
            _bmss.runSerializer(true);
        } else {
            _bmss.runSerializer(false);
            if (_bmss.RTF()) {
                _bmss.runFeeder(true);
                finTimer.start();
            } else {
                _bmss.runFeeder(false);
                finTimer.reset();
            }
        }
    }

    private void updateWidgets() {
        dsWidget.setDouble(_bmss.drumCurrentSpeed);
        hpWidget.setDouble(_bmss.hoodPosition);
        fsWidget.setBoolean(_bmss.feed);
    }

    @Override
    public void end(boolean interuppted) {
        _bmss.stop();
    }

    @Override
    public boolean isFinished() {
        return finTimer.get() > 0.5;
    }
}