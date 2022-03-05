package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.pidConstants.*;

public class DriveTo extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(TP,TI,TD);
    private final PIDController pidd = new PIDController(DP, DI, DD);
    private final Translation2d targetPoint;
    private double lastDistance = 9999999;
    private boolean done = false;
    private boolean driving = false;

    private final ShuffleboardTab tab = Shuffleboard.getTab("driveto");
    private final NetworkTableEntry distanceW = tab.add("dist",0).withSize(1,1).withPosition(0,0).getEntry();
    private final NetworkTableEntry xW = tab.add("x",0).withSize(1, 1).withPosition(0, 1).getEntry();
    private final NetworkTableEntry yW = tab.add("y",0).withSize(1, 1).withPosition(0, 2).getEntry();
    private final NetworkTableEntry htW = tab.add("HTE",0).withSize(1,1).withPosition(1, 0).getEntry();
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public DriveTo(DriveSubsystem DSS, double X, double Y) {
        _dss = DSS;
        targetPoint = new Translation2d(X,Y);
        addRequirements(_dss);
    }

    @Override
    public void initialize() {
        pidt.reset();
        pidd.reset();
        driving = false;
        lastDistance = 9999999;
    } 

    @Override
    public void execute() {
        double headingTo = headingFromTo(_dss.getPose().getTranslation(), targetPoint);
        double headErr = _dss.getHeadingError(headingTo);
        double turn = pidt.calculate(headErr);
        double drive = -1*pidd.calculate(_dss.getPose().getTranslation().getDistance(targetPoint));
        if (!driving) {
            driving = Math.abs(_dss.getHeadingError(headingTo)) < 4;
            drive = 0;
        }
        if (_dss.getPose().getTranslation().getDistance(targetPoint) > lastDistance && lastDistance < 0.5) done = true;
        else lastDistance = _dss.getPose().getTranslation().getDistance(targetPoint);
        distanceW.setDouble(lastDistance);
        xW.setDouble(_dss.getPose().getX());
        yW.setDouble(_dss.getPose().getY());
        htW.setDouble(headErr);
        _dss.arcadeDrive(drive, turn);
    }

    private double headingFromTo(Translation2d from, Translation2d to) {
        double xErr = to.minus(from).getX();
        double yErr = to.minus(from).getY()*-1;
        if (xErr == 0) {
            if (yErr > 0) return 90;
            else return 270;
        } else if (yErr == 0) {
            if (xErr > 0) return 0;
            else return 180; 
        }
        double ang = Math.toDegrees(Math.atan(yErr/xErr));
        if (yErr > 0 && ang < 0) ang += 180;
        if (yErr < 0 && ang > 0) ang -= 180;
        return ang; 
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}